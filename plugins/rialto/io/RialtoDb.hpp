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

#pragma once

#include <pdal/pdal_export.hpp>
#include <pdal/Writer.hpp>

#include <cstdint>
#include <string>


// A Rialto database contains two tables:
// 
// TileSets
//    id (PK)
//    name
//    bounds
//    levels
//    ...
//
// Tiles
//    id (PK)
//    yileset id (FK)
//    x, y, level

namespace pdal {
class Log;
class SQLite;

}

namespace rialtosupport
{

using namespace pdal;



class PDAL_DLL RialtoDb
{
public:
    enum DataType {
        Uint8, Uint16, Uint32, Uint64,
        Int8, Int16, Int32, Int64,
        Float32, Float64
    };
    
    // note we always use EPSG:4325
    struct TileSetInfo {
        std::string name; // aka filename
        uint32_t maxLevel;
        uint32_t numCols;
        uint32_t numRows;
        double minx; // tile set extents (not extents of actual data)
        double miny;
        double maxx;
        double maxy;
        uint32_t numDimensions;
    };
    
    struct DimensionInfo {
        DataType datatype;
        std::string name;
        double minimum;
        double mean;
        double maximum;
    };
    
    struct TileInfo {
        uint32_t tileSetId;
        uint32_t level;
        uint32_t x; // col
        uint32_t y; // row
    };
    
    // modes:
    //   Create: creates a new database and sets up the required tables
    //           it is an error if the db already exists
    //   Write: opens the database for writing
    //          it is an error if the db doesn't already exist
    //          it is also an error if the db doesn't have the required tables
    //   Read: opens an existing database
    //         it is an error if the db doesn't already exist
    RialtoDb(const std::string& connection);

    ~RialtoDb();
    
    void open(bool writable=false);

    void close();
    
    // adds a tile set to the database
    //
    // returns id of new data set
    uint32_t addTileSet(const RialtoDb::TileSetInfo& data);

    // remove a tile set from the database
    void deleteTileSet(uint32_t tileSetId);
    
    // get list all the tile sets in the database, as a list of its
    std::vector<uint32_t> getTileSetIds();
    
    // get info about a specific tile set
    TileSetInfo getTileSetInfo(uint32_t tileSetId);
    
    // get info about one of the dimensions of a tile set
    DimensionInfo getDimensionInfo(uint32_t tileSetId, uint32_t dimension);
    
    // returns id of new tile
    uint32_t addTile(const RialtoDb::TileInfo& data, char* buf);

    // get info about a tile
    TileInfo getTileInfo(uint32_t tileId);
    
    // query for all the points of a tile set, bounded by bbox region
    // returns a pipeline made up of a BufferReader and a CropFilter
    Stage* query(uint32_t tileSetId,
                 double minx, double miny,
                 double max, double maxy,
                 uint32_t minLevel, uint32_t maxLevel);

private:
    // query for all the tiles of a tile set, bounded by bbox region
    std::vector<uint32_t> getTileSets(uint32_t tileSetId,
                                      double minx, double miny,
                                      double max, double maxy,
                                      uint32_t minLevel, uint32_t maxLevel);

    void createTileSetsTable();
    void createTilesTable();
    
    void query();
    
    LogPtr log() const { return m_log; }
    
    std::string m_connection;
    std::unique_ptr<SQLite> m_session;
    LogPtr m_log;
    int m_srid;
    
    RialtoDb& operator=(const RialtoDb&); // not implemented
    RialtoDb(const RialtoDb&); // not implemented
};

} // namespace rialtosupport
