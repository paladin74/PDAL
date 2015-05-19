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


namespace pdal
{

namespace
{

} // anonymous namespace


RialtoDb::RialtoDb(const std::string& connection, LogPtr log) :
    m_connection(connection),
    m_log(log),
    m_srid(4326),
    m_needsIndexing(false),
    m_txStarted(false),
    e_tilesRead("tilesRead"),
    e_tileSetsRead("tileSetsRead"),
    e_tilesWritten("tilesWritten"),
    e_tileSetsWritten("tileSetsWritten"),
    e_queries("queries"),
    e_creation("creation"),
    e_indexCreation("indexCreation"),
    m_numPointsRead(0),
    m_numPointsWritten(0)
{
    //m_log->setLevel(LogLevel::Debug);
}


RialtoDb::~RialtoDb()
{
    if (m_sqlite)
    {
        close();
    }

    log()->get(LogLevel::Debug) << "~RialtoDB" << std::endl;
}


void RialtoDb::create()
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    log()->get(LogLevel::Debug) << "RialtoDB::create()" << std::endl;

    if (FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: database already exists");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(true);

    e_creation.start();
    createTileSetsTable();
    createTilesTable();
    createDimensionsTable();
    e_creation.stop();

    m_needsIndexing = true;
}


void RialtoDb::open(bool writable)
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    log()->get(LogLevel::Debug) << "RialtoDB::open()" << std::endl;

    if (!FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: database not found");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(writable);

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

    if (m_needsIndexing)
    {
        e_indexCreation.start();

        if (m_txStarted)
        {
            m_sqlite->commit();
        }

        std::ostringstream oss2;
        oss2 << "CREATE INDEX index_name ON Tiles(column,row)";
        m_sqlite->execute(oss2.str());

        std::ostringstream oss3;
        oss3 << "CREATE INDEX index_name2 ON Tiles(level)";
        m_sqlite->execute(oss3.str());

        e_indexCreation.stop();
    }

    m_sqlite.reset();

    dumpStats();
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
        << "name VARCHAR(64) NOT NULL,"           // TODO
        << "maxLevel INTEGER NOT NULL,"
        << "numDims INTEGER NOT NULL"
        << ")";

    m_sqlite->execute(oss1.str());
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
         << " tile_set_id INTEGER NOT NULL,"
         << " level INTEGER NOT NULL,"
         << " column INTEGER NOT NULL,"
         << " row INTEGER NOT NULL,"
         << " numPoints INTEGER NOT NULL,"
         << " points BLOB NOT NULL,"
         << " FOREIGN KEY(tile_set_id) REFERENCES TileSets(tile_set_id)"
//         << " UNIQUE(level,column,row)"
         << ")";

    m_sqlite->execute(oss1.str());
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
        << "tile_set_id INTEGER NOT NULL,"
        << "name VARCHAR(256) NOT NULL,"           // TODO
        << "position INTEGER NOT NULL,"
        << "dataType VARCHAR(256) NOT NULL,"
        << "minimum DOUBLE NOT NULL,"
        << "mean DOUBLE NOT NULL,"
        << "maximum DOUBLE NOT NULL,"
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

    e_tileSetsRead.start();

    std::ostringstream oss;
    oss << "SELECT tile_set_id,name,maxLevel,numDims "
        << "FROM TileSets WHERE tile_set_id=" << tileSetId;

    log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

    m_sqlite->query(oss.str());

    // should get exactly one row back
    const row* r = m_sqlite->get();
    assert(r);

    assert(tileSetId == boost::lexical_cast<uint32_t>(r->at(0).data));
    info.name = r->at(1).data;
    info.maxLevel = boost::lexical_cast<uint32_t>(r->at(2).data);
    info.numDimensions = boost::lexical_cast<uint32_t>(r->at(3).data);

    assert(!m_sqlite->next());

    info.dimensions.clear();
    info.dimensions.resize(info.numDimensions);
    readDimensionsInfo(tileSetId, info.dimensions);

    e_tileSetsRead.stop();
}


void RialtoDb::readTileInfo(uint32_t tileId, bool withPoints, RialtoDb::TileInfo& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesRead.start();

    std::ostringstream oss;
    oss << "SELECT tile_id,tile_set_id,level,column,row,numPoints"
        << (withPoints ? ",points " : " ")
        << "FROM Tiles "
        << "WHERE tile_id=" << tileId;

    //log()->get(LogLevel::Debug) << "SELECT for tile" << std::endl;

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

      ++m_numPointsRead;
    }

    e_tilesRead.stop();

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
        //log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
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

    e_tileSetsWritten.start();

    std::ostringstream oss;
    oss << "INSERT INTO TileSets "
        << "(name, maxLevel, numDims) "
        << "VALUES (?, ?, ?)";

    records rs;
    row r;

    r.push_back(column(data.name));
    r.push_back(column(data.maxLevel));
    r.push_back(column(data.numDimensions));
    rs.push_back(r);

    m_sqlite->insert(oss.str(), rs);

    long id = m_sqlite->last_row_id();
    log()->get(LogLevel::Debug) << "inserted TileSet, id=" << id << std::endl;

    writeDimensions(id, data.dimensions);

    e_tileSetsWritten.stop();

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

    e_tilesWritten.start();

    if (!m_txStarted)
    {
        m_sqlite->begin();
        m_txStarted = true;
    }

    unsigned char* buf = NULL;
    uint32_t buflen = 0;
    castPatchAsBuffer(data.patch, buf, buflen);
    assert(buf);
    assert(buflen);

    // note we don't use 'mask' in the database version of tiles
    const std::string sql =
        "INSERT INTO Tiles "
        "(tile_set_id, level, column, row, numPoints, points) "
        "VALUES (?, ?, ?, ?, ?, ?)";

    records rs;
    row r;

    r.push_back(column(data.tileSetId));
    r.push_back(column(data.level));
    r.push_back(column(data.column));
    r.push_back(column(data.row));
    r.push_back(column(data.numPoints));
    r.push_back(blob((char*)buf, (size_t)buflen));
    rs.push_back(r);

    m_sqlite->insert(sql, rs);

    long id = m_sqlite->last_row_id();
    //log()->get(LogLevel::Debug) << "inserted for tile set " << data.tileSetId
    //                            << ": tile id " << id
    //                            << "(" << data.column
    //                            << ", " << data.row
    //                            << ", " << data.level
    //                            << ")" << std::endl;

    e_tilesWritten.stop();

    m_numPointsWritten += data.numPoints;

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


void RialtoDb::xyPointToTileColRow(double x, double y, uint32_t level, uint32_t& col, uint32_t& row)
{
    //printf("---\n");

    //printf("x: %f\n", x);
    //printf("y: %f\n", y);
    //printf("l: %d\n", level);

    if (x>=180.0) x = -180.0;
    if (y<=-90.0) y = 90.0;

    double level2 = pow(2.0, level);
    //printf("l2: %f\n", level2);

    double tileWidth = (180.0 - -180.0) / level2;
    tileWidth /= 2.0;
    double tileHeight = (90.0 - -90.0) / level2;
    //printf("tileWidth: %f\n", tileWidth);
    //printf("tileHeight: %f\n", tileHeight);

    double c = (x - -180.0) / tileWidth;
    //printf("c: %f\n", c);
    col = (uint32_t)floor(c);
    //printf("col: %u\n", col);

    double r = (90.0 - y) / tileHeight;
    //printf("r: %f\n", r);
    row = (uint32_t)floor(r);
    //printf("row: %u\n", row);

    //printf("---\n");
}


void RialtoDb::queryForTileIds(uint32_t tileSetId,
                               double minx, double miny,
                               double maxx, double maxy,
                               uint32_t level,
                               std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "Querying tile set " << tileSetId
                                << " for some tile ids" << std::endl;

    ids.clear();

    e_queries.start();

    assert(minx <= maxx);
    assert(miny <= maxy);

    uint32_t mincol, minrow, maxcol, maxrow;
    xyPointToTileColRow(minx, miny, level, mincol, maxrow);
    xyPointToTileColRow(maxx, maxy, level, maxcol, minrow);

    // because boundary conditions
    if (mincol > maxcol) std::swap(mincol, maxcol);
    if (minrow > maxrow) std::swap(minrow, maxrow);

    assert(mincol <= maxcol);
    assert(minrow <= maxrow);

    std::ostringstream oss;
    oss << "SELECT tile_id FROM Tiles"
        << " WHERE tile_set_id=" << tileSetId
        << " AND level=" << level
        << " AND column >= " << mincol
        << " AND column <= " << maxcol
        << " AND row >= " << minrow
        << " AND row <= " << maxrow;

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


void RialtoDb::queryForTileInfosBegin(uint32_t tileSetId,
                                      double minx, double miny,
                                      double maxx, double maxy,
                                      uint32_t level)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesRead.start();

    log()->get(LogLevel::Debug) << "Querying tile set " << tileSetId
                                << " for some tile infos" << std::endl;

    assert(minx <= maxx);
    assert(miny <= maxy);

    uint32_t mincol, minrow, maxcol, maxrow;
    xyPointToTileColRow(minx, miny, level, mincol, maxrow);
    xyPointToTileColRow(maxx, maxy, level, maxcol, minrow);

    // because boundary conditions
    if (mincol > maxcol) std::swap(mincol, maxcol);
    if (minrow > maxrow) std::swap(minrow, maxrow);

    assert(mincol <= maxcol);
    assert(minrow <= maxrow);

    std::ostringstream oss;
    oss << "SELECT tile_id,tile_set_id,level,column,row,numPoints,points"
        << " FROM Tiles "
        << " WHERE tile_set_id=" << tileSetId
        << " AND level=" << level
        << " AND column >= " << mincol
        << " AND column <= " << maxcol
        << " AND row >= " << minrow
        << " AND row <= " << maxrow;

    m_sqlite->query(oss.str());

    e_tilesRead.stop();
}


bool RialtoDb::queryForTileInfos(TileInfo& info)
{
    e_tilesRead.start();

    const row* r = m_sqlite->get();

    if (!r) return false;

    //assert(tileId == boost::lexical_cast<uint32_t>(r->at(0).data));
    info.tileSetId = boost::lexical_cast<uint32_t>(r->at(1).data);
    info.level = boost::lexical_cast<double>(r->at(2).data);
    info.column = boost::lexical_cast<double>(r->at(3).data);
    info.row = boost::lexical_cast<double>(r->at(4).data);
    info.numPoints = boost::lexical_cast<double>(r->at(5).data);

    // this query always reads the points
    info.patch.buf.clear();
    {
        const uint32_t blobLen = r->at(6).blobLen;
        const std::vector<uint8_t>& blobBuf = r->at(6).blobBuf;
        const unsigned char *pos = (const unsigned char *)&(blobBuf[0]);
        info.patch.putBytes(pos, blobLen);
    }

    e_tilesRead.stop();

    m_numPointsRead += info.numPoints;

    return true;
}


bool RialtoDb::queryForTileInfosNext()
{
    return m_sqlite->next();
}


// appends points to end of view (does not start with point index 0)
void RialtoDb::serializeToPointView(const TileInfo& info, PointViewPtr view)
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
    }
}


void RialtoDb::setupLayout(const TileSetInfo& tileSetInfo, PointLayoutPtr layout) const
{
    for (uint32_t i=0; i<tileSetInfo.numDimensions; i++)
    {
        const DimensionInfo& dimInfo = tileSetInfo.dimensions[i];

        const Dimension::Id::Enum nameId = Dimension::id(dimInfo.name);
        const Dimension::Type::Enum typeId = Dimension::type(dimInfo.dataType);

        layout->registerDim(nameId, typeId);
    }
}


void RialtoDb::dumpStats() const
{
    e_tilesRead.dump();
    e_tileSetsRead.dump();
    e_tilesWritten.dump();
    e_tileSetsWritten.dump();
    e_queries.dump();
    e_creation.dump();
    e_indexCreation.dump();

    if (m_numPointsRead)
    {
        printf("pointsRead: %u\n", m_numPointsRead);
    }
    else
    {
        printf("pointsRead: -\n");
    }

    if (m_numPointsWritten)
    {
        printf("pointsWritten: %u\n", m_numPointsWritten);
    }
    else
    {
      printf("pointsWritten: -\n");    
    }
}

} // namespace pdal
