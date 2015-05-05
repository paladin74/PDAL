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

#include <pdal/../../plugins/sqlite/io/SQLiteCommon.hpp> // TODO: fix path


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


void RialtoDb::open(bool writable)
{
    m_log = std::shared_ptr<pdal::Log>(new pdal::Log("RialtoDB", "stdout"));
    m_log->setLevel(LogLevel::Debug);
    
    log()->get(LogLevel::Debug) << "RialtoDB::open()" << std::endl;

    m_session = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
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

    createTileSetsTable();
    createTilesTable();
}


void RialtoDb::close()
{
    // TODO
}


void RialtoDb::createTileSetsTable()
{
    if (m_session->doesTableExist("TileSets")) return;

    std::ostringstream oss1;
    std::ostringstream oss2;

    oss1 << "CREATE TABLE TileSets("
        << "tile_set_id INTEGER PRIMARY KEY AUTOINCREMENT,"
        << "name VARCHAR(64),"           // TODO
        << "minx DOUBLE,"
        << "miny DOUBLE,"
        << "maxx DOUBLE,"
        << "maxy DOUBLE,"
        << "maxLevels INTEGER"
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
    if (m_session->doesTableExist("Tile")) return;

    std::ostringstream oss1;

    oss1 << "CREATE TABLE Tiles("
        << "tile_id INTEGER PRIMARY KEY AUTOINCREMENT,"
        << "tile_set_id INTEGER, "
        << "level INTEGER,"
        << "x DOUBLE,"
        << "y DOUBLE,"
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


uint32_t RialtoDb::addTileSet(const std::string& name)
{
    std::ostringstream oss;
    oss << "INSERT INTO TileSets "
        << "(name, minx, miny, maxx, maxy, maxLevels) "
        << "VALUES (?, ?, ?, ?, ?, ?)";

    records rs;
    row r;
    
    r.push_back(column(name));
    r.push_back(column(22));
    r.push_back(column(33));
    r.push_back(column(44));
    r.push_back(column(55));
    r.push_back(column(66));
    rs.push_back(r);

//    m_session->insert(oss.str(), rs);

    long id = 0;//m_session->last_row_id();
//    log()->get(LogLevel::Debug) << "inserted id=" << id << std::endl;
    
    return id;
}


} // namespace rialtosupport
