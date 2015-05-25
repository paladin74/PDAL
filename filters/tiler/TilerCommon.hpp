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

#include <pdal/pdal_types.hpp>
#include <pdal/Writer.hpp>

#include <cassert>
#include <cstdint>
#include <vector>

namespace pdal
{

namespace tilercommon
{


// the one, true class for the math operations on a tile matrix
//
// - a Tile is a rectangular region of space, covering a bbox
// - a Tile Matrix is a 2D array of tiles
// - a Tile Matrix has nc columns and nr rows and a level l
// - tiles are numbered from (0,0) at upper left
// - a Tile Matrix Set is the set of Tile Matrices for nl levels, 0<=l<nl
// - a bbox is defined by the points (minx,miny) and (maxx,maxy), inclusive
//   of the lower bound and exclusive of the upper bound
// - point space has (0,0) at lower left
// - tiles are defined with uint32s
// - points are defined with doubles
// - all methods are designed to be inlined
// - all methods are const
// - all methods are O(1) operations
// - if arguments to an operation are out of bounds, results are undefined
class TileMatrixMath
{
public:
    enum Quad
    {
        QuadSW=0, QuadNW=1, QuadSE=2, QuadNE=3,
    };

    TileMatrixMath(double minx, double miny, double maxx, double maxy,
                   uint32_t numColsAtL0, uint32_t numRowsAtL0) :
    m_minx(minx),
    m_miny(miny),
    m_maxx(maxx),
    m_maxy(maxy),
    m_nc0(numColsAtL0),
    m_nr0(numRowsAtL0)
    {
        assert(minx < maxx);
        assert(miny < maxy);
        assert(numColsAtL0 > 0);
        assert(numRowsAtL0 > 0);
    }

    // bounds of the matrix
    double minX() const { return m_minx; }
    double maxX() const { return m_maxx; }
    double minY() const { return m_miny; }
    double maxY() const { return m_maxy; }

    // does this matrix contain the point?
    bool matrixContains(double x, double y) const
    {
        return contains(m_minx, m_miny, m_maxx, m_maxy, x, y);
    }

    uint32_t numColsAtLevel(uint32_t level) const
    {
        return ipow2(level) * m_nc0;
    }

    uint32_t numRowsAtLevel(uint32_t level) const
    {
        return ipow2(level) * m_nr0;
    }

    double tileWidthAtLevel(uint32_t level) const
    {
        return (m_maxx - m_minx) / numColsAtLevel(level);
    }

    double tileHeightAtLevel(uint32_t level) const
    {
        return (m_maxy - m_miny) / numRowsAtLevel(level);
    }

    // lower bound of a tile (c,r,l)
    void getTileBounds(uint32_t col, uint32_t row, uint32_t level,
                    double& minx, double& miny,
                    double& maxx, double& maxy) const
    {
        const double w = tileWidthAtLevel(level);
        const double h = tileHeightAtLevel(level);
        minx = m_minx + col * w;
        maxx = minx + w;
        maxy = m_maxy - row * h;
        miny = maxy - h;
    }

    void getTileOfPoint(double x, double y, uint32_t level,
                        uint32_t& col, uint32_t& row) const
    {
        const double w = tileWidthAtLevel(level);
        const double h = tileHeightAtLevel(level);
        col = std::floor((x - m_minx) / w);
        row = std::floor((m_maxy - y) / h) - 1;
    }

    bool tileContains(uint32_t col, uint32_t row, uint32_t level,
                      double x, double y) const
    {
        double minx, miny, maxx, maxy;
        getTileBounds(col, row, level, minx, miny, maxx, maxy);
        return contains(minx, miny, maxx, maxy, x, y);
    }

    void getChildOfTile(uint32_t col, uint32_t row, Quad q,
                        uint32_t& childCol, uint32_t& childRow) const
    {
        childCol = col * 2;
        childRow = row * 2;
        if (q == QuadNW)
        {
           // ok
        }
        else if (q == QuadNE)
        {
           ++childCol;
        }
        else if (q == QuadSW)
        {
            ++childRow;
        }
        else
        {
            assert(q == QuadSE);
            ++childCol;
            ++childRow;
        }
    }

    void getParentOfTile(uint32_t col, uint32_t row,
                         uint32_t& parentCol, uint32_t& parentRow) const
    {
        // integer division rounds down
        parentCol = col / 2;
        parentRow = row / 2;
    }

    Quad getQuadrant(uint32_t col, uint32_t row, uint32_t level,
                     double x, double y) const
    {
        double minx, miny, maxx, maxy;
        getTileBounds(col, row, level, minx, miny, maxx, maxy);

        const double w = tileWidthAtLevel(level);
        const double h = tileHeightAtLevel(level);
        const double midx = minx + w/2.0;
        const double midy = miny + h/2.0;

        if (contains(minx, miny, midx, midy, x, y)) return QuadSW;
        if (contains(midx, miny, maxx, midy, x, y)) return QuadSE;
        if (contains(minx, midy, midx, maxy, x, y)) return QuadNW;
        assert(contains(midx, midy, maxx, maxy, x, y));
        return QuadNE;
    }

private:
    // computes 2^n
    //
    // note that 2^0 == 1
    static uint32_t ipow2(uint32_t n)
    {
      return 1u << n;
    }

    static bool contains(double minx, double miny,
                         double maxx, double maxy,
                         double x, double y)
    {
        return (x >= minx && x < maxx && y >= miny && y < maxy);
    }

    const double m_minx, m_miny, m_maxx, m_maxy;
    const uint32_t m_nc0, m_nr0;
};


class Tile;


// the tile set holds all the tiles that make up the tile tree
//
// (note the tree is not technically a tree as it has two roots,
// one on each side of Greenwich)
class TileSet
{
    public:
        TileSet(uint32_t maxLevel, LogPtr log);
        ~TileSet();

        void ready(PointTableRef);
        void done(PointTableRef);
        void run(PointViewPtr sourceView, PointViewSet* outputSet);

        uint32_t getMaxLevel() const { return m_maxLevel; }

        PointViewPtr createPointView();
        LogPtr log() { return m_log; }

        uint32_t newTileId() { uint32_t t = m_tileId; ++m_tileId; return t; }

        const TileMatrixMath tmm;

    private:
        void setHeaderMetadata();
        void setStatisticsMetadata();

        PointViewPtr m_sourceView;
        PointViewSet* m_outputSet;
        uint32_t m_maxLevel;
        LogPtr m_log;
        Tile** m_roots;
        uint32_t m_tileId;

        MetadataNode m_tableMetadata;
        MetadataNode m_tileSetMetadata;
};


// A node of the tile tree
//
// Tiles are identified by level, column (x), and row (y)
// A tile may contain points; if so, m_pointView will be set.
// A tile may have 0..3 children.
class Tile
{
public:
    Tile(TileSet& tileSet, uint32_t level, uint32_t column, uint32_t row);
    ~Tile();

    void add(PointViewPtr pointView, PointId pointNumber, double lon, double lat);
    void setTileMetadata(uint32_t* data) const;
    void setMask();

private:
    LogPtr log() { return m_tileSet.log(); }
    char* getPointData(const PointView& buf, PointId& idx) const;
    void setMaskMetadata();

    uint32_t m_id;
    TileSet& m_tileSet;
    uint32_t m_level;
    uint32_t m_column;
    uint32_t m_row;
    Tile** m_children;
    uint64_t m_skip;
    PointViewPtr m_pointView;
    uint32_t m_mask;
};

} // namespace tilercommon
} // namespace pdal
