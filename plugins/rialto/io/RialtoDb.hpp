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

#include "RialtoSupport.hpp"


namespace pdal
{
    class Log;
    class SQLite;

namespace rialto
{
    

class PDAL_DLL RialtoDb
{
public:
    // pass it the filename of the sqlite db
    RialtoDb(const std::string& connection, LogPtr log);

    ~RialtoDb();

    void create();

    void open(bool writable=false);

    void close();

    // adds a tile set to the database, including its dimensions
    //
    // returns id of new data set
    void writeTileTable(const TileTableInfo& data);

    // returns id of new tile
    void writeTile(const std::string& tileTableName, const TileInfo& data);

    // get list all the tile sets in the database, as a list of its
    void readTileTableNames(std::vector<std::string>& names) const;

    // get info about a specific tile set (including its dimensions)
    void readTileTable(std::string const& name, TileTableInfo& info) const;

    // get info about a tile
    void readTile(std::string const& name, uint32_t tileId, bool withPoints, TileInfo& tileInfo) const;

    // use with caution for levels greater than 16 or so
    // DANGER: this assumes only one tile set per database, use only for testing
    // yes this returns the tile ids (table's PK)
    void readTileIdsAtLevel(std::string const& name, uint32_t level, std::vector<uint32_t>& tileIds) const;

    // query for all the tiles of a tile set, bounded by bbox region
    // returns the tile ids (PKs from table)
    void queryForTileIds(std::string const& name,
                         double minx, double miny,
                         double maxx, double maxy,
                         uint32_t level,
                         std::vector<uint32_t>& ids) const;

     // combines query-for-tile-ids with query-for-tile-info
     void queryForTiles_begin(std::string const& name,
                             double minx, double miny,
                             double maxx, double maxy,
                             uint32_t level);
     bool queryForTiles_step(TileInfo& tileInfo);
     bool queryForTiles_next();

     uint32_t querySrsId(const std::string& wkt) const;
     std::string querySrsWkt(uint32_t srs_id) const;

    // fills in the dimensions of an otherwise empty layout with
    // the dimension information from the tile set
    void setupLayout(const TileTableInfo& tileTableInfo, PointLayoutPtr layout) const;

     static void xyPointToTileColRow(double x, double y, uint32_t level, uint32_t& col, uint32_t& row);

     void dumpStats() const;

private:
    void verifyTableExists(std::string const& name) const;
    
    void createGpkgId();
    void createTableGpkgSpatialRefSys();
    void createTableGpkgContents();
    void createTableGpkgPctileMatrixSet();
    void createTableGpkgPctileMatrix();
    void createTableGpkgPctile(const std::string& table_name);
    void createTableGpkgMetadata();
    void createTableGpkgMetadataReference();
    void createTableGpkgExtensions();
    void createTablePctilesDimensionSet();

    void writeDimensions(const TileTableInfo&);
    void writeMetadata(const TileTableInfo&);

    // get info about one of the dimensions of a tile set
    void readDimensions(std::string const& name, std::vector<DimensionInfo>&) const;

    void matrixSizeAtLevel(uint32_t level, uint32_t& numCols, uint32_t& numRows) const;

    LogPtr log() const { return m_log; }

    std::string m_connection;
    std::unique_ptr<SQLite> m_sqlite;
    LogPtr m_log;
    int m_srid;
    bool m_needsIndexing;
    bool m_txStarted;
    bool m_haveWrittenMatrixAtLevel[32]; // TODO: 32 is highest possible level
    
    mutable RialtoEvent e_tilesRead;
    mutable RialtoEvent e_tileTablesRead;
    mutable RialtoEvent e_tilesWritten;
    mutable RialtoEvent e_tileTablesWritten;
    mutable RialtoEvent e_queries;
    mutable RialtoEvent e_creation;
    mutable RialtoEvent e_indexCreation;
    mutable uint32_t m_numPointsRead;
    mutable uint32_t m_numPointsWritten;

    RialtoDb& operator=(const RialtoDb&); // not implemented
    RialtoDb(const RialtoDb&); // not implemented
};

} // namespace rialto

} // namespace pdal
