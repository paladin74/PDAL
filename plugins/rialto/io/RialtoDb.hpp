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

#include <pdal/pdal.hpp>

// TODO: just used for Patch? (SQLite can be fwd declared)
#include "../plugins/sqlite/io/SQLiteCommon.hpp" // TODO: fix path

#include "RialtoEvent.hpp"

namespace pdal {
    class Log;
    class SQLite;
}

namespace pdal
{

using namespace pdal;



class PDAL_DLL RialtoDb
{
public:
    struct DimensionInfo {
        std::string name;
        uint32_t position;
        std::string dataType;
        std::string description;
        double minimum;
        double mean;
        double maximum;
    };

    // Rialto has some hard-coded restrictions:
    //   we always use EPSG:4326
    //   we always start with two tiles at the root
    //   we always cover the whole globe at the root
    //   we always do power-of-two reductions
    //   we store all levels between 0 and max, inclusive
    struct TileSetInfo {
        std::string datetime;
        std::string name; // aka filename
        uint32_t maxLevel;
        uint32_t numDimensions;
        std::vector<DimensionInfo> dimensions;
        double data_min_x; // data extents
        double data_min_y;
        double data_max_x;
        double data_max_y;
        double tmset_min_x; // tile extents
        double tmset_min_y;
        double tmset_max_x;
        double tmset_max_y;
    };

    struct TileInfo {
        uint32_t tileSetId;
        uint32_t level;
        uint32_t column;
        uint32_t row;
        uint32_t numPoints; // used in database, but not on disk version
        uint32_t mask; // used in disk version, but not in database
        Patch patch;
    };

    // pass it the filename of the sqlite db
    RialtoDb(const std::string& connection, LogPtr log);

    ~RialtoDb();

    void create();

    void open(bool writable=false);

    void close();

    // adds a tile set to the database, including its dimensions
    //
    // returns id of new data set
    uint32_t writeTileSet(const RialtoDb::TileSetInfo& data);

    // returns id of new tile
    uint32_t writeTile(const std::string& tileSetName, const RialtoDb::TileInfo& data);

    // get list all the tile sets in the database, as a list of its
    void readTileSetIds(std::vector<uint32_t>&, std::vector<std::string>& names) const;

    // get info about a specific tile set (including its dimensions)
    void readTileSetInfo(uint32_t tileSetId, std::string const& name, TileSetInfo& info) const;

    // get info about a tile
    void readTileInfo(uint32_t tileId, bool withPoints, TileInfo& tileInfo) const;

    // use with caution for levels greater than 16 or so
    void readTileIdsAtLevel(uint32_t tileSetId, uint32_t level, std::vector<uint32_t>& tileIds) const;

    // query for all the tiles of a tile set, bounded by bbox region
    void queryForTileIds(uint32_t tileSetId,
                         double minx, double miny,
                         double maxx, double maxy,
                         uint32_t level,
                         std::vector<uint32_t>& ids) const;

     // combines query-for-tile-ids with query-for-tile-info
     void queryForTileInfosBegin(uint32_t tileSetId, std::string const& name,
                                 double minx, double miny,
                                 double maxx, double maxy,
                                 uint32_t level);
     bool queryForTileInfos(TileInfo& tileInfo);
     bool queryForTileInfosNext();

    // fills in the dimensions of an otherwise empty layout with
    // the dimension information from the tile set
    void setupLayout(const TileSetInfo& tileSetInfo, PointLayoutPtr layout) const;

     // just hides the type punning
     static void castPatchAsBuffer(const Patch&, unsigned char*& buf, uint32_t& bufLen);

     static void xyPointToTileColRow(double x, double y, uint32_t level, uint32_t& col, uint32_t& row);

     void serializeToPointView(const TileInfo& info, PointViewPtr view);

     void dumpStats() const;

private:
    // create the req'd tables in the db
    void createTileSetsTable();
    void createTableGpkgContents();
    void createTilesTable();
    void createDimensionsTable();

    void createGpkgId();
    void createTableGpkgSpatialRefSys();
    void createTableGpkgPctileMatrixSet();
    void createTableGpkgPctileMatrix();
    void createTableTilePyramidUserData(const std::string& table_name);
    void createTableGpkgMetadata();
    void createTableGpkgMetadataReference();
    void createTableGpkgExtensions();
    void createTablePctilesDimensionSet();

    // add all the dimensions of the tile set
    void writeDimensions(uint32_t tileSetId,
                        const std::vector<DimensionInfo>& dimensions);

    // get info about one of the dimensions of a tile set
    void readDimensionsInfo(uint32_t tileSetId, std::string const& name, std::vector<DimensionInfo>&) const;

    void matrixSizeAtLevel(uint32_t level, uint32_t& numCols, uint32_t& numRows) const;

    void query() const;

    LogPtr log() const { return m_log; }

    std::string m_connection;
    std::unique_ptr<SQLite> m_sqlite;
    LogPtr m_log;
    int m_srid;
    bool m_needsIndexing;
    bool m_txStarted;
    bool m_haveWrittenMatrixAtLevel[32]; // TODO: 32 is highest possible level
    
    mutable RialtoEvent e_tilesRead;
    mutable RialtoEvent e_tileSetsRead;
    mutable RialtoEvent e_tilesWritten;
    mutable RialtoEvent e_tileSetsWritten;
    mutable RialtoEvent e_queries;
    mutable RialtoEvent e_creation;
    mutable RialtoEvent e_indexCreation;
    mutable uint32_t m_numPointsRead;
    mutable uint32_t m_numPointsWritten;

    RialtoDb& operator=(const RialtoDb&); // not implemented
    RialtoDb(const RialtoDb&); // not implemented
};

} // namespace pdal
