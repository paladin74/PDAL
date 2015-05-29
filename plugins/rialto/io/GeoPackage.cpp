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

#include "GeoPackage.hpp"
#include "GeoPackageCommon.hpp"

#include <../plugins/sqlite/io/SQLiteCommon.hpp>
#include <../filters/tiler/TilerCommon.hpp>


namespace pdal
{

namespace rialto
{


GeoPackage::GeoPackage(const std::string& connection, LogPtr log) :
    m_connection(connection),
    m_log(log),
    e_readMatrixSet("readMatrixSet"),
    e_srsQueries("srsQueries")

{
    //m_log->setLevel(LogLevel::Debug);
}


GeoPackage::~GeoPackage()
{
    assert(!m_sqlite);

    log()->get(LogLevel::Debug) << "~RialtoDB" << std::endl;
}


void GeoPackage::internalOpen(bool writable)
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    log()->get(LogLevel::Debug) << "RialtoDB::create()" << std::endl;

    if (!writable && !FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: does not exist");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(writable);
}


void GeoPackage::internalClose()
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    m_sqlite.reset();
}


uint32_t GeoPackage::querySrsId(const std::string& wkt) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_srsQueries.start();

    // TODO: how better to look up SRS than text match of WKT?

    std::ostringstream oss;
    oss << "SELECT srs_id FROM gpkg_spatial_ref_sys"
        << " WHERE definition='" << wkt << "'";

    m_sqlite->query(oss.str());

    e_srsQueries.stop();

    const row* r = m_sqlite->get();
    if (!r)
    {
        return 0;
    }

    // take the first one, ignore any others
    const uint32_t srs_id = boost::lexical_cast<uint32_t>(r->at(0).data);

    return srs_id;
}


std::string GeoPackage::querySrsWkt(uint32_t srs_id) const
{
    e_srsQueries.start();

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

    e_srsQueries.stop();
    return wkt;
}


bool GeoPackage::doesTableExist(std::string const& name) const
{
    return m_sqlite->doesTableExist(name);
}


void GeoPackage::verifyTableExists(std::string const& name) const
{
    if (!doesTableExist(name))
    {
        throw pdal_error("required GeoPackage table not found: " + name);
    }
}


void GeoPackage::beginTransaction()
{
    m_sqlite->begin();
}


void GeoPackage::commitTransaction()
{
    m_sqlite->commit();
}


void GeoPackage::readMatrixSet(std::string const& name, GpkgMatrixSet& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_readMatrixSet.start();

    std::string datetime;
    uint32_t srs_id;
    double data_min_x, data_min_y, data_max_x, data_max_y;
    std::string description;
    {
        std::ostringstream oss;
        oss << "SELECT last_change, min_x, min_y, max_x, max_y, srs_id, description "
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
        description = r->at(6).data;
        assert(!m_sqlite->next());
    }

    double tmset_min_x, tmset_min_y, tmset_max_x, tmset_max_y;
    {
        // tile extents, not data extents
        std::ostringstream oss;
        oss << "SELECT min_x, min_y, max_x, max_y "
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

    uint32_t numColsAtL0, numRowsAtL0;
    {
        std::ostringstream oss;
        oss << "SELECT matrix_width, matrix_height"
            << " FROM gpkg_pctile_matrix"
            << " WHERE table_name='" << name << "'"
            << " AND zoom_level=0";

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        numColsAtL0 = boost::lexical_cast<uint32_t>(r->at(0).data);
        numRowsAtL0 = boost::lexical_cast<uint32_t>(r->at(1).data);
        assert(!m_sqlite->next());
    }
    assert(numColsAtL0==2);
    assert(numRowsAtL0==1);

    uint32_t fileId;
    {
        // TODO: should be a JOIN, just one query
        
        std::ostringstream oss1;
        oss1 << "SELECT md_file_id FROM gpkg_metadata_reference"
            << " WHERE table_name='" << name << "'";

        m_sqlite->query(oss1.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        fileId = boost::lexical_cast<uint32_t>(r->at(0).data);
        assert(!m_sqlite->next());
    }
    
    std::string lasMetadata;
    {
        std::ostringstream oss2;
        oss2 << "SELECT metadata FROM gpkg_metadata"
            << " WHERE id=" << fileId;

        m_sqlite->query(oss2.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        lasMetadata = r->at(0).data;
        assert(!m_sqlite->next());
    }
    
    info.set(datetime, name, maxLevel, numDimensions, wkt,
             data_min_x, data_min_y, data_max_x, data_max_y,
             tmset_min_x, tmset_min_y, tmset_max_x, tmset_max_y,
             numColsAtL0, numRowsAtL0, description, lasMetadata);

    readDimensions(info.getName(), info.getDimensionsRef());
    assert(info.getDimensions().size() == info.getNumDimensions());

    e_readMatrixSet.stop();
}


void GeoPackage::readMatrixSetNames(std::vector<std::string>& names) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    const std::string sql("SELECT table_name FROM gpkg_contents WHERE data_type = 'pctiles'");

    log()->get(LogLevel::Debug1) << "SELECT for tile set ids" << std::endl;

    names.clear();

    m_sqlite->query(sql);

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        column const& c = r->at(0);
        const std::string name = c.data;
        log()->get(LogLevel::Debug1) << " got name: " << name << std::endl;
        names.push_back(name);

    } while (m_sqlite->next());
}


void GeoPackage::readMatrixSets(std::vector<GpkgMatrixSet>& infos) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    // first get all the table names
    std::vector<std::string> names;
    readMatrixSetNames(names);

    infos.resize(names.size());

    // now, get the info for each table
    for (uint32_t i=0; i<names.size(); i++)
    {
        GpkgMatrixSet& info = infos[i];
        readMatrixSet(names[i], info);
    }
}


void GeoPackage::readDimensions(std::string const& name, std::vector<GpkgDimension>& dimensionsInfo) const
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


void GeoPackage::dumpStats() const
{
    childDumpStats();
    e_srsQueries.dump();
    e_readMatrixSet.dump();
}


} // namespace rialto
} // namespace pdal
