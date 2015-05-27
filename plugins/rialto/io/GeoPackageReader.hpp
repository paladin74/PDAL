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

#include "GeoPackage.hpp"
#include "Event.hpp"


namespace pdal
{
    class Log;
    class SQLite;

namespace rialto
{
    
class GpkgMatrixSet;
class GpkgTile;
class GpkgDimension;


class PDAL_DLL GeoPackageReader : public GeoPackage
{
public:
    // pass it the filename of the sqlite db
    GeoPackageReader(const std::string& connection, LogPtr log);

    virtual ~GeoPackageReader();

    virtual void open();
    virtual void close();

    // get info about a tile
    void readTile(std::string const& name, uint32_t tileId, bool withPoints, GpkgTile& tileInfo) const;

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
     bool queryForTiles_step(GpkgTile& tileInfo);
     bool queryForTiles_next();

     virtual void dumpStats() const;

     // fills in the dimensions of an otherwise empty layout with
     // the dimension information from the tile set
     static void setupLayout(const GpkgMatrixSet& tileTableInfo, PointLayoutPtr layout);

private:
    int m_srid;
    
    mutable Event e_tilesRead;
    mutable Event e_tileTablesRead;
    mutable Event e_queries;
    mutable uint32_t m_numPointsRead;

    GeoPackageReader& operator=(const GeoPackageReader&); // not implemented
    GeoPackageReader(const GeoPackageReader&); // not implemented
};


} // namespace rialto

} // namespace pdal
