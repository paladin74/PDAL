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

#include "RialtoDb.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_error.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/BufferReader.hpp>
#include "../filters/crop/CropFilter.hpp" // TODO: fix path

#include <cstdint>

#include <../plugins/sqlite/io/SQLiteCommon.hpp> // TODO: fix path


// A Rialto database contains these tables:
//
// TileSets
//    tile_set_id (PK)
//    name
//    maxLevel
//    numCols
//    numRows
//    minx
//    miny
//    maxx
//    maxy
//    numDims
//
// Tiles
//    tile_id (PK)
//    tile_set_id (FK)
//    level
//    column
//    row
//    points
//
// Dimensions
//    tile_set_id (FK)
//    name
//    position
//    datatype
//    minimum
//    mean
//    maximum


namespace rialtosupport
{

namespace
{

} // anonymous namespace


RialtoDb::RialtoDb(const std::string& connection) :
    m_connection(connection),
    m_srid(4326)
{
}


RialtoDb::~RialtoDb()
{
    log()->get(LogLevel::Debug) << "~RialtoDB" << std::endl;
}


void RialtoDb::create()
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }
    
    m_log = std::shared_ptr<pdal::Log>(new pdal::Log("RialtoDB", "stdout"));
    m_log->setLevel(LogLevel::Debug);

    log()->get(LogLevel::Debug) << "RialtoDB::open()" << std::endl;

    if (FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: database already exists");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(true);

#if 0
    m_sqlite->spatialite();

    const bool hasSpatialite = m_sqlite->doesTableExist("geometry_columns");
    if (!hasSpatialite)
    {
       std::ostringstream oss;
       oss << "SELECT InitSpatialMetadata()";
       m_sqlite->execute(oss.str());
    }
#endif

    createTileSetsTable();
    createTilesTable();
    createDimensionsTable();
}


void RialtoDb::open(bool writable)
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    m_log = std::shared_ptr<pdal::Log>(new pdal::Log("RialtoDB", "stdout"));
    m_log->setLevel(LogLevel::Debug);

    log()->get(LogLevel::Debug) << "RialtoDB::open()" << std::endl;

    if (!FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: database not found");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(writable);

#if 0
    m_sqlite->spatialite();

    const bool hasSpatialite = m_sqlite->doesTableExist("geometry_columns");
    if (!hasSpatialite)
    {
       std::ostringstream oss;
       oss << "SELECT InitSpatialMetadata()";
       m_sqlite->execute(oss.str());
    }
#endif

    if (!m_sqlite->doesTableExist("TileSets"))
        throw pdal_error("RialtoDb: required table 'TileSets' not found");
    if (!m_sqlite->doesTableExist("Tiles"))
        throw pdal_error("RialtoDb: required table 'Tiles' not found");
    if (!m_sqlite->doesTableExist("Dimensions"))
        throw pdal_error("RialtoDb: required table 'Dimensions' not found");
}


void RialtoDb::close()
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    m_sqlite.reset();
}


void RialtoDb::createTileSetsTable()
{
    if (m_sqlite->doesTableExist("TileSets"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'TileSets' already exists)");
    }

    if (m_sqlite->doesTableExist("TileSets")) return;

    std::ostringstream oss1;
    std::ostringstream oss2;

    oss1 << "CREATE TABLE TileSets("
        << "tile_set_id INTEGER PRIMARY KEY AUTOINCREMENT,"
        << "name VARCHAR(64),"           // TODO
        << "maxLevel INTEGER,"
        << "numCols INTEGER,"
        << "numRows INTEGER,"
        << "minx DOUBLE,"
        << "miny DOUBLE,"
        << "maxx DOUBLE,"
        << "maxy DOUBLE,"
        << "numDims INTEGER"
        << ")";

    m_sqlite->execute(oss1.str());

#if 0
    oss2 << "SELECT AddGeometryColumn('TileSets', 'extent', "
        << m_srid << ", "
        << "'POLYGON', 'XY')";
    m_sqlite->execute(oss2.str());
#endif
}


void RialtoDb::createTilesTable()
{
    if (m_sqlite->doesTableExist("Tiles"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'Tiles' already exists)");
    }

    std::ostringstream oss1;

    oss1 << "CREATE TABLE Tiles("
        << "tile_id INTEGER PRIMARY KEY AUTOINCREMENT,"
        << "tile_set_id INTEGER, "
        << "level INTEGER,"
        << "column INTEGER,"
        << "row INTEGER,"
        << "numPoints INTEGER,"
        << "points BLOB, "
        << "FOREIGN KEY(tile_set_id) REFERENCES TileSets(tile_set_id)"
        << ")";

    m_sqlite->execute(oss1.str());

#if 0
    std::ostringstream oss2;
    oss2 << "SELECT AddGeometryColumn('Tiles', 'extent', "
        << m_srid << ", "
        << "'POLYGON', 'XY')";
    m_sqlite->execute(oss2.str());
#endif
}


void RialtoDb::createDimensionsTable()
{
    if (m_sqlite->doesTableExist("Dimensions"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'Dimensions' already exists)");
    }

    if (m_sqlite->doesTableExist("Dimensions")) return;

    std::ostringstream oss1;
    std::ostringstream oss2;

    oss1 << "CREATE TABLE Dimensions("
        << "tile_set_id INTEGER,"
        << "name VARCHAR(256),"           // TODO
        << "position INTEGER,"
        << "dataType VARCHAR(256),"
        << "minimum DOUBLE,"
        << "mean DOUBLE,"
        << "maximum DOUBLE,"
        << "FOREIGN KEY(tile_set_id) REFERENCES TileSets(tile_set_id)"
        << ")";

    m_sqlite->execute(oss1.str());
}


void RialtoDb::readTileSetIds(std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    ids.clear();
    
    std::ostringstream oss;
    oss << "SELECT tile_set_id FROM TileSets";

    log()->get(LogLevel::Debug1) << "SELECT for tile set ids" << std::endl;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        column const& c = r->at(0);
        const uint32_t id = boost::lexical_cast<uint32_t>(c.data);
        log()->get(LogLevel::Debug1) << " got id: " << id << std::endl;
        ids.push_back(id);

    } while (m_sqlite->next());
}


void RialtoDb::readTileSetInfo(uint32_t tileSetId, TileSetInfo& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::ostringstream oss;
    oss << "SELECT tile_set_id,name,maxLevel,numCols,numRows,minx,miny,maxx,maxy,numDims "
        << "FROM TileSets WHERE tile_set_id=" << tileSetId;

    log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

    m_sqlite->query(oss.str());

    // should get exactly one row back
    const row* r = m_sqlite->get();
    assert(r);

    assert(tileSetId == boost::lexical_cast<uint32_t>(r->at(0).data));
    info.name = r->at(1).data;
    info.maxLevel = boost::lexical_cast<uint32_t>(r->at(2).data);
    info.numCols = boost::lexical_cast<uint32_t>(r->at(3).data);
    info.numRows = boost::lexical_cast<uint32_t>(r->at(4).data);
    info.minx = boost::lexical_cast<double>(r->at(5).data);
    info.miny = boost::lexical_cast<double>(r->at(6).data);
    info.maxx = boost::lexical_cast<double>(r->at(7).data);
    info.maxy = boost::lexical_cast<double>(r->at(8).data);
    info.numDimensions = boost::lexical_cast<uint32_t>(r->at(9).data);
    
    assert(info.numCols == 2);
    assert(info.numRows == 1);

    assert(!m_sqlite->next());

    info.dimensions.clear();
    info.dimensions.resize(info.numDimensions);
    readDimensionsInfo(tileSetId, info.dimensions);
}


void RialtoDb::readTileInfo(uint32_t tileId, bool withPoints, RialtoDb::TileInfo& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::ostringstream oss;
    oss << "SELECT tile_id,tile_set_id,level,column,row,numPoints"
        << (withPoints ? ",points " : " ")
        << "FROM Tiles "
        << "WHERE tile_id=" << tileId;

    log()->get(LogLevel::Debug) << "SELECT for tile" << std::endl;

    m_sqlite->query(oss.str());

    // should get exactly one row back
    const row* r = m_sqlite->get();
    assert(r);

    assert(tileId == boost::lexical_cast<uint32_t>(r->at(0).data));
    info.tileSetId = boost::lexical_cast<uint32_t>(r->at(1).data);
    info.level = boost::lexical_cast<double>(r->at(2).data);
    info.column = boost::lexical_cast<double>(r->at(3).data);
    info.row = boost::lexical_cast<double>(r->at(4).data);
    info.numPoints = boost::lexical_cast<double>(r->at(5).data);

    info.patch.buf.clear();
    if (withPoints)
    {
      const uint32_t blobLen = r->at(6).blobLen;
      const std::vector<uint8_t>& blobBuf = r->at(6).blobBuf;
      const unsigned char *pos = (const unsigned char *)&(blobBuf[0]);
      info.patch.putBytes(pos, blobLen);
    }

    assert(!m_sqlite->next());
}


void RialtoDb::readTileIdsAtLevel(uint32_t tileSetId, uint32_t level, std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    ids.clear();
    
    std::ostringstream oss;
    oss << "SELECT tile_id FROM Tiles"
        << " WHERE tile_set_id=" << tileSetId
        << " AND level=" << level;

    log()->get(LogLevel::Debug) << "SELECT for tile ids at level: " << level << std::endl;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_sqlite->next());
}


void RialtoDb::readDimensionsInfo(uint32_t tileSetId, std::vector<DimensionInfo>& dimensionsInfo) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    dimensionsInfo.clear();
    
    std::ostringstream oss;
    oss << "SELECT tile_set_id,name,position,dataType,minimum,mean,maximum "
        << "FROM Dimensions "
        << "WHERE tile_set_id=" << tileSetId;

    log()->get(LogLevel::Debug) << "SELECT for dim info of tile set " << tileSetId << std::endl;

    m_sqlite->query(oss.str());

    int i = 0;
    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        DimensionInfo& info = dimensionsInfo[i];
        
        assert(tileSetId == boost::lexical_cast<uint32_t>(r->at(0).data));
        info.name = r->at(1).data;
        info.position = boost::lexical_cast<double>(r->at(2).data);
        info.dataType = r->at(3).data;
        info.minimum = boost::lexical_cast<double>(r->at(4).data);
        info.mean = boost::lexical_cast<double>(r->at(5).data);
        info.maximum = boost::lexical_cast<double>(r->at(6).data);

        log()->get(LogLevel::Debug1) << "read dim: " << info.name << std::endl;
        
        ++i;
    } while (m_sqlite->next());
}


uint32_t RialtoDb::writeTileSet(const RialtoDb::TileSetInfo& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "RialtoDb::addTileSet()" << std::endl;

    std::ostringstream oss;
    oss << "INSERT INTO TileSets "
        << "(name, maxLevel, numCols, numRows, minx, miny, maxx, maxy, numDims) "
        << "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";

    records rs;
    row r;

    r.push_back(column(data.name));
    r.push_back(column(data.maxLevel));
    r.push_back(column(data.numCols));
    r.push_back(column(data.numRows));
    r.push_back(column(data.minx));
    r.push_back(column(data.miny));
    r.push_back(column(data.maxx));
    r.push_back(column(data.maxy));
    r.push_back(column(data.numDimensions));
    rs.push_back(r);

    m_sqlite->insert(oss.str(), rs);

    long id = m_sqlite->last_row_id();
    log()->get(LogLevel::Debug) << "inserted TileSet, id=" << id << std::endl;
    log()->get(LogLevel::Debug) << "     " << data.numCols << data.numRows << std::endl;

    writeDimensions(id, data.dimensions);
    
    return id;
}


void RialtoDb::writeDimensions(uint32_t tileSetId,
                             const std::vector<DimensionInfo>& dims)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::ostringstream oss;
    oss << "INSERT INTO Dimensions "
      << "(tile_set_id, name, position, dataType, minimum, mean, maximum) "
      << "VALUES (?, ?, ?, ?, ?, ?, ?)";

    for (auto dim: dims)
    {
        records rs;
        row r;

        log()->get(LogLevel::Debug) << "INSERT for dim: " << dim.name << std::endl;

        r.push_back(column(tileSetId));
        r.push_back(column(dim.name.c_str()));
        r.push_back(column(dim.position));
        r.push_back(column(dim.dataType));
        r.push_back(column(dim.minimum));
        r.push_back(column(dim.mean));
        r.push_back(column(dim.maximum));
        rs.push_back(r);

        m_sqlite->insert(oss.str(), rs);
    }
}


uint32_t RialtoDb::writeTile(const RialtoDb::TileInfo& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    unsigned char* buf = NULL;
    uint32_t buflen = 0;
    castPatchAsBuffer(data.patch, buf, buflen);
    assert(buf);
    assert(buflen);

    // note we don't use 'mask' in the database version of tiles
    std::ostringstream oss;
    oss << "INSERT INTO Tiles "
        << "(tile_set_id, level, column, row, numPoints, points) "
        << "VALUES (?, ?, ?, ?, ?, ?)";

    records rs;
    row r;

    r.push_back(column(data.tileSetId));
    r.push_back(column(data.level));
    r.push_back(column(data.column));
    r.push_back(column(data.row));
    r.push_back(column(data.numPoints));
    r.push_back(blob((char*)buf, (size_t)buflen));
    rs.push_back(r);

    m_sqlite->insert(oss.str(), rs);

    long id = m_sqlite->last_row_id();
    log()->get(LogLevel::Debug) << "inserted for tile set " << data.tileSetId
                                << ": tile id " << id << std::endl;

    return id;
}


void RialtoDb::castPatchAsBuffer(const Patch& patch, unsigned char*& buf, uint32_t& bufLen)
{
    buf = NULL;
    bufLen = patch.buf.size();    
    if (bufLen) {
        buf = (unsigned char*)&patch.buf[0];
    }
}


void RialtoDb::queryForTileIds(uint32_t tileSetId,
                               double minx, double miny,
                               double max, double maxy,
                               uint32_t level,
                               std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "Querying tile set " << tileSetId
                                << " for some tile ids" << std::endl;

    // TODO: for now, just return *all* the tiles at the level
    ids.clear();
    
    std::ostringstream oss;
    oss << "SELECT tile_id FROM Tiles"
        << " WHERE tile_set_id=" << tileSetId
        << " AND level=" << level;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_sqlite->next());
}


// appends points to end of view (does not start with point index 0)
static void serializeToPointView(const rialtosupport::RialtoDb::TileInfo& info, PointViewPtr view, LogPtr log)
{
    const size_t numPoints = info.numPoints;
    PointId idx = view->size();
    const uint32_t pointSize = view->pointSize();
    
    const Patch& patch = info.patch;
    
    const char* buf = (const char*)(&patch.buf[0]);
    const DimTypeList& dtl = view->dimTypes();
    for (size_t i=0; i<numPoints; ++i)
    {
        view->setPackedPoint(dtl, idx, buf);
        buf += pointSize;
        ++idx;
        
        log->get(LogLevel::Debug) << "here" << std::endl;
    }
}


void RialtoDb::setupPointTable(uint32_t tileSetId, PointTable& table) const
{
    // TODO: this should all be done in 1 query
    
    TileSetInfo tileSetInfo;
    readTileSetInfo(tileSetId, tileSetInfo);
    
    for (uint32_t i=0; i<tileSetInfo.numDimensions; i++)
    {
        const DimensionInfo& dimInfo = tileSetInfo.dimensions[i];

        const Dimension::Id::Enum nameId = Dimension::id(dimInfo.name);
        const Dimension::Type::Enum typeId = Dimension::type(dimInfo.dataType);
        
        table.layout()->registerDim(nameId, typeId);
    }        
}


Stage* RialtoDb::query(PointTable& table,
                       uint32_t tileSetId,
                       double minx, double miny,
                       double maxx, double maxy,
                       uint32_t level) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "RialtoDb::querying tile set " << tileSetId
                                << "  at level " << level
                                << " with bounds ("
                                << minx << ","
                                << miny << ","
                                << maxx << ","
                                << maxy << ")"
                                << std::endl;

    std::vector<uint32_t> ids;
    queryForTileIds(tileSetId, minx, miny, maxx, maxy, level, ids);

    PointViewPtr view(new PointView(table));

    uint32_t pointIndex = 0;
    for (auto id: ids) 
    {
        TileInfo info;
        readTileInfo(id, true, info);
        
        log()->get(LogLevel::Debug) << "  got some points: " << info.numPoints << std::endl;

        serializeToPointView(info, view, log());

        log()->get(LogLevel::Debug) << "  view now has this many: " << view->size() << std::endl;
    }
    
    Options readerOptions;
    BufferReader* reader = new BufferReader(); // TODO: needs ptr
    reader->setOptions(readerOptions);
    reader->addView(view);
    reader->setSpatialReference(SpatialReference("EPSG:4326"));

    // TODO: we set Z bounds because BOX3D::compare(), used inside the crop
    // filter, will get it wrong if we don't
    const double minz = (std::numeric_limits<double>::lowest)();
    const double maxz = (std::numeric_limits<double>::max)();
    BOX3D dstBounds(minx, miny, minz, maxx, maxy, maxz);
    Options cropOpts;
    cropOpts.add("bounds", dstBounds);
    cropOpts.add("verbose", LogLevel::Debug5);

    CropFilter* cropper = new CropFilter(); // TODO: needs ptr
    cropper->setOptions(cropOpts);
    cropper->setInput(*reader);

    return cropper;
}


} // namespace rialtosupport
