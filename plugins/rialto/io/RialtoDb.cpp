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
//    x
//    y
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
    m_session(NULL),
    m_srid(4326)
{
}


RialtoDb::~RialtoDb()
{
    log()->get(LogLevel::Debug) << "~RialtoDB" << std::endl;
}


void RialtoDb::create()
{
    if (m_session)
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

    m_session = new SQLite(m_connection, m_log);
    m_session->connect(true);

#if 0
    m_session->spatialite();

    const bool hasSpatialite = m_session->doesTableExist("geometry_columns");
    if (!hasSpatialite)
    {
       std::ostringstream oss;
       oss << "SELECT InitSpatialMetadata()";
       m_session->execute(oss.str());
    }
#endif

    createTileSetsTable();
    createTilesTable();
    createDimensionsTable();
}


void RialtoDb::open(bool writable)
{
    if (m_session)
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

    m_session = new SQLite(m_connection, m_log);
    m_session->connect(writable);

#if 0
    m_session->spatialite();

    const bool hasSpatialite = m_session->doesTableExist("geometry_columns");
    if (!hasSpatialite)
    {
       std::ostringstream oss;
       oss << "SELECT InitSpatialMetadata()";
       m_session->execute(oss.str());
    }
#endif

    if (!m_session->doesTableExist("TileSets"))
        throw pdal_error("RialtoDb: required table 'TileSets' not found");
    if (!m_session->doesTableExist("Tiles"))
        throw pdal_error("RialtoDb: required table 'Tiles' not found");
    if (!m_session->doesTableExist("Dimensions"))
        throw pdal_error("RialtoDb: required table 'Dimensions' not found");
}


void RialtoDb::close()
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    delete m_session;
    m_session = NULL;
}


void RialtoDb::createTileSetsTable()
{
    if (m_session->doesTableExist("TileSets"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'TileSets' already exists)");
    }

    if (m_session->doesTableExist("TileSets")) return;

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

    m_session->execute(oss1.str());

#if 0
    oss2 << "SELECT AddGeometryColumn('TileSets', 'extent', "
        << m_srid << ", "
        << "'POLYGON', 'XY')";
    m_session->execute(oss2.str());
#endif
}


void RialtoDb::createTilesTable()
{
    if (m_session->doesTableExist("Tiles"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'Tiles' already exists)");
    }

    std::ostringstream oss1;

    oss1 << "CREATE TABLE Tiles("
        << "tile_id INTEGER PRIMARY KEY AUTOINCREMENT,"
        << "tile_set_id INTEGER, "
        << "level INTEGER,"
        << "x INTEGER,"
        << "y INTEGER,"
        << "numPoints INTEGER,"
        << "points BLOB, "
        << "FOREIGN KEY(tile_set_id) REFERENCES TileSets(tile_set_id)"
        << ")";

    m_session->execute(oss1.str());

#if 0
    std::ostringstream oss2;
    oss2 << "SELECT AddGeometryColumn('Tiles', 'extent', "
        << m_srid << ", "
        << "'POLYGON', 'XY')";
    m_session->execute(oss2.str());
#endif
}


void RialtoDb::createDimensionsTable()
{
    if (m_session->doesTableExist("Dimensions"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'Dimensions' already exists)");
    }

    if (m_session->doesTableExist("Dimensions")) return;

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

    m_session->execute(oss1.str());
}


std::vector<uint32_t> RialtoDb::getTileSetIds()
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::vector<uint32_t> ids;

    std::ostringstream oss;
    oss << "SELECT tile_set_id FROM TileSets";

    log()->get(LogLevel::Debug1) << "SELECT for tile set ids" << std::endl;

    m_session->query(oss.str());

    do {
        const row* r = m_session->get();
        if (!r) break;

        column const& c = r->at(0);
        const uint32_t id = boost::lexical_cast<uint32_t>(c.data);
        log()->get(LogLevel::Debug1) << " got id: " << id << std::endl;
        ids.push_back(id);

    } while (m_session->next());

    return ids;
}


RialtoDb::TileSetInfo RialtoDb::getTileSetInfo(uint32_t tileSetId)
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    TileSetInfo info;

    std::ostringstream oss;
    oss << "SELECT tile_set_id,name,maxLevel,numCols,numRows,minx,miny,maxx,maxy,numDims "
        << "FROM TileSets WHERE tile_set_id=" << tileSetId;

    log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

    m_session->query(oss.str());

    // should get exactly one row back
    const row* r = m_session->get();
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

    assert(!m_session->next());

    return info;
}


RialtoDb::TileInfo RialtoDb::getTileInfo(uint32_t tileId, bool withPoints)
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    TileInfo info;

    std::ostringstream oss;
    oss << "SELECT tile_id,level,x,y,numPoints"
        << (withPoints ? ",points " : " ")
        << "FROM Tiles "
        << "WHERE tile_id=" << tileId;

    log()->get(LogLevel::Debug) << "SELECT for tile" << std::endl;

    m_session->query(oss.str());

    // should get exactly one row back
    const row* r = m_session->get();
    assert(r);

    assert(tileId == boost::lexical_cast<uint32_t>(r->at(0).data));
    info.level = boost::lexical_cast<double>(r->at(1).data);
    info.x = boost::lexical_cast<double>(r->at(2).data);
    info.y = boost::lexical_cast<double>(r->at(3).data);
    info.numPoints = boost::lexical_cast<double>(r->at(4).data);

    info.patch.buf.clear();
    if (withPoints)
    {
      const uint32_t blobLen = r->at(5).blobLen;
      const std::vector<uint8_t>& blobBuf = r->at(5).blobBuf;
      const unsigned char *pos = (const unsigned char *)&(blobBuf[0]);
      info.patch.putBytes(pos, blobLen);
    }

    assert(!m_session->next());

    return info;
}


std::vector<uint32_t> RialtoDb::getTileIdsAtLevel(uint32_t tileSetId, uint32_t level)
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::vector<uint32_t> ids;

    std::ostringstream oss;
    oss << "SELECT tile_id FROM Tiles"
        << " WHERE tile_set_id=" << tileSetId
        << " AND level=" << level;

    log()->get(LogLevel::Debug) << "SELECT for tile ids at level: " << level << std::endl;

    m_session->query(oss.str());

    do {
        const row* r = m_session->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_session->next());

    return ids;
}


RialtoDb::DimensionInfo RialtoDb::getDimensionInfo(uint32_t tileSetId, uint32_t position)
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    DimensionInfo info;

    std::ostringstream oss;
    oss << "SELECT tile_set_id,name,position,dataType,minimum,mean,maximum "
        << "FROM Dimensions "
        << "WHERE tile_set_id=" << tileSetId
        << " AND position=" << position;

    log()->get(LogLevel::Debug) << "SELECT for dim info, position: " << position << std::endl;

    m_session->query(oss.str());

    // should get exactly one row back
    const row* r = m_session->get();
    assert(r);

    assert(tileSetId == boost::lexical_cast<uint32_t>(r->at(0).data));
    info.name = r->at(1).data;
    assert(position == boost::lexical_cast<uint32_t>(r->at(2).data));
    info.dataType = r->at(3).data;
    info.minimum = boost::lexical_cast<double>(r->at(4).data);
    info.mean = boost::lexical_cast<double>(r->at(5).data);
    info.maximum = boost::lexical_cast<double>(r->at(6).data);

    assert(!m_session->next());

    return info;
}


uint32_t RialtoDb::addTileSet(const RialtoDb::TileSetInfo& data)
{
    if (!m_session)
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

    m_session->insert(oss.str(), rs);

    long id = m_session->last_row_id();
    log()->get(LogLevel::Debug) << "inserted TileSet, id=" << id << std::endl;
    log()->get(LogLevel::Debug) << "     " << data.numCols << data.numRows << std::endl;

    return id;
}


void RialtoDb::addDimensions(uint32_t tileSetId,
                             const std::vector<DimensionInfo>& dims)
{
    if (!m_session)
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

    m_session->insert(oss.str(), rs);
    }
}


uint32_t RialtoDb::addTile(const RialtoDb::TileInfo& data)
{
    if (!m_session)
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
        << "(tile_set_id, level, x, y, numPoints, points) "
        << "VALUES (?, ?, ?, ?, ?, ?)";

    records rs;
    row r;

    r.push_back(column(data.tileSetId));
    r.push_back(column(data.level));
    r.push_back(column(data.x));
    r.push_back(column(data.y));
    r.push_back(column(data.numPoints));
    r.push_back(blob((char*)buf, (size_t)buflen));
    rs.push_back(r);

    m_session->insert(oss.str(), rs);

    long id = m_session->last_row_id();
    log()->get(LogLevel::Debug) << "inserted Tile, id=" << id << std::endl;

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


std::vector<uint32_t> RialtoDb::queryForTileIds(uint32_t tileSetId,
                                                double minx, double miny,
                                                double max, double maxy,
                                                uint32_t level)
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::vector<uint32_t> ids;

    // for now, just return all the tiles at the level
    
    std::ostringstream oss;
    oss << "SELECT tile_id FROM Tiles"
        << " WHERE tile_set_id=" << tileSetId
        << " AND level=" << level;

    log()->get(LogLevel::Debug) << "SELECT for tile query" << std::endl;

    m_session->query(oss.str());

    do {
        const row* r = m_session->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_session->next());

    return ids;
}


Stage* RialtoDb::query(uint32_t tileSetId,
                       double minx, double miny,
                       double maxx, double maxy,
                       uint32_t level)
{
    if (!m_session)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::vector<uint32_t> ids = queryForTileIds(tileSetId, minx, miny,
                                                maxx, maxy, level);
    if (!ids.size()) return NULL;

    PointTable table;
    PointViewPtr inputView(new PointView(table));

    table.layout()->registerDim(Dimension::Id::X);  // TODO
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    TileInfo info;
    for (auto id: ids) 
    {
        getTileInfo(id, true);
    }
    
    assert(0);
    return NULL;
}


} // namespace rialtosupport
