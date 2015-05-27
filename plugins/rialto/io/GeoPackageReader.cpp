/******************************************************************************
* Copyright (c) 2015, RadiantBlue Technologies, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "GeoPackageReader.hpp"
#include "GeoPackageCommon.hpp"

#include <pdal/../../plugins/sqlite/io/SQLiteCommon.hpp> // TODO: fix path
#include <pdal/../../filters/tiler/TilerCommon.hpp> // TODO: fix path


namespace pdal
{

namespace rialto
{

GeoPackageReader::GeoPackageReader(const std::string& connection, LogPtr log) :
    GeoPackage(connection, log),
    m_srid(4326),
    e_tilesRead("tilesRead"),
    e_tileTablesRead("tileTablesRead"),
    e_queries("queries"),
    m_numPointsRead(0)
{
}


GeoPackageReader::~GeoPackageReader()
{
}


void GeoPackageReader::open()
{
    internalOpen(false);

    verifyTableExists("gpkg_spatial_ref_sys");
    verifyTableExists("gpkg_contents");
    verifyTableExists("gpkg_pctile_matrix");
    verifyTableExists("gpkg_pctile_matrix_set");
    verifyTableExists("gpkg_extensions");
    verifyTableExists("gpkg_metadata");
    verifyTableExists("gpkg_metadata_reference");
}


void GeoPackageReader::close()
{
    internalClose();
}


void GeoPackageReader::setupLayout(const GpkgMatrixSet& tileTableInfo, PointLayoutPtr layout)
{
    for (uint32_t i=0; i<tileTableInfo.getNumDimensions(); i++)
    {
        const GpkgDimension& dimInfo = tileTableInfo.getDimensions()[i];

        const Dimension::Id::Enum nameId = Dimension::id(dimInfo.getName());
        const Dimension::Type::Enum typeId = Dimension::type(dimInfo.getDataType());

        layout->registerDim(nameId, typeId);
    }
}


void GeoPackageReader::readTileTableNames(std::vector<std::string>& names) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    names.clear();

    std::ostringstream oss;
    oss << "SELECT table_name FROM gpkg_contents";

    log()->get(LogLevel::Debug1) << "SELECT for tile set ids" << std::endl;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        column const& c = r->at(0);
        const std::string name = c.data;
        log()->get(LogLevel::Debug1) << " got name: " << name << std::endl;
        names.push_back(name);

    } while (m_sqlite->next());
}


void GeoPackageReader::readTileTable(std::string const& name, GpkgMatrixSet& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tileTablesRead.start();

    std::string datetime;
    uint32_t srs_id;
    double data_min_x, data_min_y, data_max_x, data_max_y;
    {
        std::ostringstream oss;
        oss << "SELECT last_change, data_min_x, data_min_y, data_max_x, data_max_y, srs_id "
            << "FROM gpkg_contents WHERE table_name='" << name << "'";

        log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        datetime = r->at(0).data;
        data_min_x = boost::lexical_cast<double>(r->at(1).data);
        data_min_y = boost::lexical_cast<double>(r->at(2).data);
        data_max_x = boost::lexical_cast<double>(r->at(3).data);
        data_max_y = boost::lexical_cast<double>(r->at(4).data);
        srs_id = boost::lexical_cast<uint32_t>(r->at(5).data);
        assert(!m_sqlite->next());
    }

    double tmset_min_x, tmset_min_y, tmset_max_x, tmset_max_y;
    {
        std::ostringstream oss;
        oss << "SELECT tmset_min_x, tmset_min_y, tmset_max_x, tmset_max_y "
            << "FROM gpkg_pctile_matrix_set WHERE table_name='" << name << "'";

        log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        tmset_min_x = boost::lexical_cast<double>(r->at(0).data);
        tmset_min_y = boost::lexical_cast<double>(r->at(1).data);
        tmset_max_x = boost::lexical_cast<double>(r->at(2).data);
        tmset_max_y = boost::lexical_cast<double>(r->at(3).data);
        assert(!m_sqlite->next());
    }

    std::string wkt = querySrsWkt(srs_id);

    uint32_t maxLevel;
    {
        std::ostringstream oss;
        oss << "SELECT MAX(zoom_level) "
            << "FROM gpkg_pctile_matrix WHERE table_name='" << name << "'";

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        maxLevel = boost::lexical_cast<uint32_t>(r->at(0).data);
        assert(!m_sqlite->next());
    }

    uint32_t numDimensions;
    {
        std::ostringstream oss;
        oss << "SELECT COUNT(table_name) "
            << "FROM pctiles_dimension_set WHERE table_name='" << name << "'";

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        numDimensions = boost::lexical_cast<uint32_t>(r->at(0).data);
        assert(numDimensions != 0);
        assert(!m_sqlite->next());
    }

    info.set(datetime, name, maxLevel, numDimensions, wkt,
             data_min_x, data_min_y, data_max_x, data_max_y,
             tmset_min_x, tmset_min_y, tmset_max_x, tmset_max_y);

    readDimensions(info.getName(), info.getDimensionsRef());
    assert(info.getDimensions().size() == info.getNumDimensions());

    e_tileTablesRead.stop();
}


void GeoPackageReader::readTile(std::string const& name, uint32_t tileId, bool withPoints, GpkgTile& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesRead.start();

    std::ostringstream oss;
    oss << "SELECT zoom_level,tile_column,tile_row,num_points,child_mask"
        << (withPoints ? ",tile_data " : " ")
        << "FROM '" << name << "'"
        << "WHERE id=" << tileId;

    //log()->get(LogLevel::Debug) << "SELECT for tile" << std::endl;

    m_sqlite->query(oss.str());

    // should get exactly one row back
    const row* r = m_sqlite->get();
    assert(r);

    const uint32_t level = boost::lexical_cast<uint32_t>(r->at(0).data);
    const uint32_t column = boost::lexical_cast<uint32_t>(r->at(1).data);
    const uint32_t row = boost::lexical_cast<uint32_t>(r->at(2).data);
    const uint32_t numPoints = boost::lexical_cast<uint32_t>(r->at(3).data);
    const uint32_t mask = boost::lexical_cast<uint32_t>(r->at(4).data);
    info.set(level, column, row, numPoints, mask);

    info.getPatchRef().clear();
    if (withPoints)
    {
        const std::vector<uint8_t>& v = r->at(5).blobBuf;
        info.getPatchRef().importFromVector(v);
        ++m_numPointsRead;
    }

    e_tilesRead.stop();

    assert(!m_sqlite->next());
}


void GeoPackageReader::readTileIdsAtLevel(std::string const& name, uint32_t level, std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    ids.clear();

    std::ostringstream oss;
    oss << "SELECT id FROM '" << name << "'"
        << " WHERE zoom_level=" << level;

    log()->get(LogLevel::Debug) << "SELECT for tile ids at level: " << level << std::endl;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        //log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_sqlite->next());
}


void GeoPackageReader::readDimensions(std::string const& name, std::vector<GpkgDimension>& dimensionsInfo) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    dimensionsInfo.clear();

    std::ostringstream oss;
    oss << "SELECT ordinal_position, dimension_name, data_type, description, minimum, mean, maximum "
        << "FROM pctiles_dimension_set WHERE table_name='" << name << "'";

    m_sqlite->query(oss.str());

    int i = 0;
    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        const uint32_t position = boost::lexical_cast<uint32_t>(r->at(0).data);
        const std::string name = r->at(1).data;
        const std::string dataType = r->at(2).data;
        const std::string description = r->at(3).data;
        const double minimum = boost::lexical_cast<double>(r->at(4).data);
        const double mean = boost::lexical_cast<double>(r->at(5).data);
        const double maximum = boost::lexical_cast<double>(r->at(6).data);

        GpkgDimension info(name, position, dataType, description, minimum, mean, maximum);

        log()->get(LogLevel::Debug1) << "read dim: " << info.getName() << std::endl;

        ++i;

        dimensionsInfo.push_back(info);
    } while (m_sqlite->next());
}


std::string GeoPackageReader::querySrsWkt(uint32_t srs_id) const
{
    std::ostringstream oss;
    oss << "SELECT definition "
        << "FROM gpkg_spatial_ref_sys WHERE srs_id=" << srs_id;

    log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

    m_sqlite->query(oss.str());

    // should get exactly one row back
    const row* r = m_sqlite->get();
    assert(r);
    const std::string wkt = r->at(0).data;
    assert(!m_sqlite->next());
    return wkt;
}


void GeoPackageReader::queryForTileIds(std::string const& name,
                               double minx, double miny,
                               double maxx, double maxy,
                               uint32_t level,
                               std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "Querying tile set " << name
                                << " for some tile ids" << std::endl;

    ids.clear();

    e_queries.start();

    assert(minx <= maxx);
    assert(miny <= maxy);

    // TODO: hard-coded for 4326
    const tilercommon::TileMatrixMath tmm(-180.0, -90.0, 180.0, 90.0, 2, 1);
    uint32_t mincol, minrow, maxcol, maxrow;
    tmm.getTileOfPoint(minx, miny, level, mincol, minrow);
    tmm.getTileOfPoint(maxx, maxy, level, maxcol, maxrow);

    // because grid starts upper-left
    std::swap(minrow, maxrow);

    assert(mincol <= maxcol);
    assert(minrow <= maxrow);

    std::ostringstream oss;
    oss << "SELECT id FROM '" << name << "'"
        << " WHERE zoom_level=" << level
        << " AND tile_column >= " << mincol
        << " AND tile_column <= " << maxcol
        << " AND tile_row >= " << minrow
        << " AND tile_row <= " << maxrow;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_sqlite->next());

    e_queries.stop();
}


void GeoPackageReader::queryForTiles_begin(std::string const& name,
                                   double minx, double miny,
                                   double maxx, double maxy,
                                   uint32_t level)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesRead.start();

    log()->get(LogLevel::Debug) << "Querying tile set " << name
                                << " for some tile infos" << std::endl;

    assert(minx <= maxx);
    assert(miny <= maxy);

    // TODO: hard-coded for 4326
    const tilercommon::TileMatrixMath tmm(-180.0, -90.0, 180.0, 90.0, 2, 1);
    uint32_t mincol, minrow, maxcol, maxrow;
    // we use mincol/maxrow and maxcol/minrow because the tile matrix has (0,0) at upper-left
    tmm.getTileOfPoint(minx, miny, level, mincol, maxrow);
    tmm.getTileOfPoint(maxx, maxy, level, maxcol, minrow);

    assert(mincol <= maxcol);
    assert(minrow <= maxrow);

    std::ostringstream oss;
    oss << "SELECT zoom_level,tile_column,tile_row,num_points,child_mask,tile_data"
        << " FROM '" << name << "'"
        << " WHERE zoom_level=" << level
        << " AND tile_column >= " << mincol
        << " AND tile_column <= " << maxcol
        << " AND tile_row >= " << minrow
        << " AND tile_row <= " << maxrow;

    m_sqlite->query(oss.str());

    e_tilesRead.stop();
}


bool GeoPackageReader::queryForTiles_step(GpkgTile& info)
{
    e_tilesRead.start();

    const row* r = m_sqlite->get();

    if (!r)
    {
        e_tilesRead.stop();
        return false;
    }

    const uint32_t level = boost::lexical_cast<double>(r->at(0).data);
    const uint32_t column = boost::lexical_cast<double>(r->at(1).data);
    const uint32_t row = boost::lexical_cast<double>(r->at(2).data);
    const uint32_t numPoints = boost::lexical_cast<double>(r->at(3).data);
    const uint32_t mask = boost::lexical_cast<double>(r->at(4).data);
    info.set(level, column, row, numPoints, mask);

    // this query always reads the points
    const std::vector<uint8_t>& v = r->at(5).blobBuf;
    info.getPatchRef().importFromVector(v);

    e_tilesRead.stop();

    m_numPointsRead += info.getNumPoints();

    return true;
}


bool GeoPackageReader::queryForTiles_next()
{
    return m_sqlite->next();
}



void GeoPackageReader::dumpStats() const
{
    e_tilesRead.dump();
    e_tileTablesRead.dump();
    e_queries.dump();

    if (m_numPointsRead)
    {
        printf("pointsRead: %u\n", m_numPointsRead);
    }
    else
    {
        printf("pointsRead: -\n");
    }
}


} // namespace rialto
} // namespace pdal
