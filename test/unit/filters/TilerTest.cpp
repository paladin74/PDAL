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

#include <pdal/pdal_test_main.hpp>

#include <pdal/BufferReader.hpp>
#include <TilerFilter.hpp>
#include <StatsFilter.hpp>

#include "Support.hpp"

using namespace pdal;


static void checkMath(const tilercommon::TileMatrixMath& tmm,
                      uint32_t e_col, uint32_t e_row, uint32_t e_level,
                      double e_minx, double e_miny, double e_maxx, double e_maxy)
{
    using namespace tilercommon;

    double a_minx, a_miny, a_maxx, a_maxy;
    uint32_t a_col, a_row, a_level;

    const double e_maxx_eps = e_maxx - 0.0001 * (e_maxx - e_minx);
    const double e_maxy_eps = e_maxy - 0.0001 * (e_maxy - e_miny);

    tmm.getTileBounds(e_col, e_row, e_level, a_minx, a_miny, a_maxx, a_maxy);

    EXPECT_DOUBLE_EQ(e_minx, a_minx);
    EXPECT_DOUBLE_EQ(e_miny, a_miny);
    EXPECT_DOUBLE_EQ(e_maxx, a_maxx);
    EXPECT_DOUBLE_EQ(e_maxy, a_maxy);

    tmm.getTileOfPoint(e_minx, e_miny, e_level, a_col, a_row);
    EXPECT_EQ(e_col, a_col);
    EXPECT_EQ(e_row, a_row);

    EXPECT_TRUE(tmm.tileContains(e_col, e_row, e_level, e_minx, e_miny));
    EXPECT_FALSE(tmm.tileContains(e_col, e_row, e_level, e_minx, e_maxy));
    EXPECT_FALSE(tmm.tileContains(e_col, e_row, e_level, e_maxx, e_miny));
    EXPECT_FALSE(tmm.tileContains(e_col, e_row, e_level, e_maxx, e_maxy));

    EXPECT_EQ(TileMatrixMath::QuadSW, tmm.getQuadrant(e_col, e_row, e_level, e_minx, e_miny));
    EXPECT_EQ(TileMatrixMath::QuadNW, tmm.getQuadrant(e_col, e_row, e_level, e_minx, e_maxy_eps));
    EXPECT_EQ(TileMatrixMath::QuadSE, tmm.getQuadrant(e_col, e_row, e_level, e_maxx_eps, e_miny));
    EXPECT_EQ(TileMatrixMath::QuadNE, tmm.getQuadrant(e_col, e_row, e_level, e_maxx_eps, e_maxy_eps));
}


static void checkChildren(const tilercommon::TileMatrixMath& tmm,
                          uint32_t col, uint32_t row, uint32_t level,
                          uint32_t col_nw, uint32_t row_nw,
                          uint32_t col_ne, uint32_t row_ne,
                          uint32_t col_sw, uint32_t row_sw,
                          uint32_t col_se, uint32_t row_se)
{
    using namespace tilercommon;

    uint32_t a_col, a_row;

    tmm.getChildOfTile(col, row, TileMatrixMath::QuadNW, a_col, a_row);
    EXPECT_EQ(a_col, col_nw);
    EXPECT_EQ(a_row, row_nw);

    tmm.getChildOfTile(col, row, TileMatrixMath::QuadNE, a_col, a_row);
    EXPECT_EQ(a_col, col_ne);
    EXPECT_EQ(a_row, row_ne);

    tmm.getChildOfTile(col, row, TileMatrixMath::QuadSW, a_col, a_row);
    EXPECT_EQ(a_col, col_sw);
    EXPECT_EQ(a_row, row_sw);

    tmm.getChildOfTile(col, row, TileMatrixMath::QuadSE, a_col, a_row);
    EXPECT_EQ(a_col, col_se);
    EXPECT_EQ(a_row, row_se);

    tmm.getParentOfTile(col_nw, row_nw, a_col, a_row);
    EXPECT_EQ(a_col, col);
    EXPECT_EQ(a_row, row);

    tmm.getParentOfTile(col_ne, row_ne, a_col, a_row);
    EXPECT_EQ(a_col, col);
    EXPECT_EQ(a_row, row);

    tmm.getParentOfTile(col_sw, row_sw, a_col, a_row);
    EXPECT_EQ(a_col, col);
    EXPECT_EQ(a_row, row);

    tmm.getParentOfTile(col_se, row_se, a_col, a_row);
    EXPECT_EQ(a_col, col);
    EXPECT_EQ(a_row, row);
}


TEST(TilerTest, test_tiler_matrix_math_one)
{
    // 4326
    // two cols, one row
    // (-180,-90) to (180,90)

    using namespace tilercommon;

    const TileMatrixMath tmm(-180.0, -90.0, 180.0, 90.0, 2u, 1u);

    EXPECT_DOUBLE_EQ(tmm.minX(), -180.0);
    EXPECT_DOUBLE_EQ(tmm.minY(), -90.0);
    EXPECT_DOUBLE_EQ(tmm.maxX(), 180.0);
    EXPECT_DOUBLE_EQ(tmm.maxY(), 90.0);

    EXPECT_TRUE(tmm.matrixContains(-180.0, -90.0));
    EXPECT_FALSE(tmm.matrixContains(-180.0, 90.0));
    EXPECT_FALSE(tmm.matrixContains(180.0, -90.0));
    EXPECT_FALSE(tmm.matrixContains(180.0, 90.0));

    EXPECT_EQ(tmm.numColsAtLevel(0), 2u);
    EXPECT_EQ(tmm.numRowsAtLevel(0), 1u);
    EXPECT_EQ(tmm.numColsAtLevel(1), 4u);
    EXPECT_EQ(tmm.numRowsAtLevel(1), 2u);
    EXPECT_EQ(tmm.numColsAtLevel(2), 8u);
    EXPECT_EQ(tmm.numRowsAtLevel(2), 4u);

    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(0), 180.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(0), 180.0);
    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(1), 90.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(1), 90.0);
    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(2), 45.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(2), 45.0);

    checkMath(tmm, 0, 0, 0, -180, -90, 0, 90);
    checkMath(tmm, 1, 0, 0, 0, -90, 180, 90);

    checkMath(tmm, 0, 0, 1, -180, 0, -90, 90);
    checkMath(tmm, 1, 0, 1, -90, 0, 0, 90);
    checkMath(tmm, 2, 0, 1, 0, 0, 90, 90);
    checkMath(tmm, 3, 0, 1, 90, 0, 180, 90);
    checkMath(tmm, 0, 1, 1, -180, -90, -90, 0);
    checkMath(tmm, 1, 1, 1, -90, -90, 0, 0);
    checkMath(tmm, 2, 1, 1, -0, -90, 90, 0);
    checkMath(tmm, 3, 1, 1, 90, -90, 180, 0);

    uint32_t c, r;
    double x, y;
    for (c=0, x=-180; c<8; c++, x+=45)
    {
        for (r=0, y=90-45; r<4; r++, y+=-45)
        {
          checkMath(tmm, c, r, 2, x, y, x+45, y+45);
        }
    }

    checkChildren(tmm, 0, 0, 0,
                  0, 0, 1, 0, 0, 1, 1, 1);
    checkChildren(tmm, 1, 0, 0,
                  2, 0, 3, 0, 2, 1, 3, 1);

    checkChildren(tmm, 0, 0, 1,
                  0, 0, 1, 0, 0, 1, 1, 1);
    checkChildren(tmm, 1, 0, 1,
                  2, 0, 3, 0, 2, 1, 3, 1);
    checkChildren(tmm, 2, 0, 1,
                  4, 0, 5, 0, 4, 1, 5, 1);
    checkChildren(tmm, 3, 0, 1,
                  6, 0, 7, 0, 6, 1, 7, 1);
    checkChildren(tmm, 0, 1, 1,
                  0, 2, 1, 2, 0, 3, 1, 3);
    checkChildren(tmm, 1, 1, 1,
                  2, 2, 3, 2, 2, 3, 3, 3);
    checkChildren(tmm, 2, 1, 1,
                  4, 2, 5, 2, 4, 3, 5, 3);
    checkChildren(tmm, 3, 1, 1,
                  6, 2, 7, 2, 6, 3, 7, 3);

}

TEST(TilerTest, test_tiler_matrix_math_two)
{
    // one col by one row
    // (10,2000) to (100,3000)

    using namespace tilercommon;

    const TileMatrixMath tmm(10.0, 2000.0, 100.0, 3000.0, 1u, 1u);

    EXPECT_DOUBLE_EQ(tmm.minX(), 10.0);
    EXPECT_DOUBLE_EQ(tmm.minY(), 2000.0);
    EXPECT_DOUBLE_EQ(tmm.maxX(), 100.0);
    EXPECT_DOUBLE_EQ(tmm.maxY(), 3000.0);

    EXPECT_TRUE(tmm.matrixContains(10.0, 2000.0));
    EXPECT_FALSE(tmm.matrixContains(10.0, 3000.0));
    EXPECT_FALSE(tmm.matrixContains(100.0, 2000.0));
    EXPECT_FALSE(tmm.matrixContains(100.0, 3000.0));

    EXPECT_EQ(tmm.numColsAtLevel(0), 1u);
    EXPECT_EQ(tmm.numRowsAtLevel(0), 1u);
    EXPECT_EQ(tmm.numColsAtLevel(1), 2u);
    EXPECT_EQ(tmm.numRowsAtLevel(1), 2u);
    EXPECT_EQ(tmm.numColsAtLevel(2), 4u);
    EXPECT_EQ(tmm.numRowsAtLevel(2), 4u);

    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(0), 90.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(0), 1000.0);
    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(1), 45.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(1), 500.0);
    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(2), 22.5);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(2), 250.0);

    checkMath(tmm, 0, 0, 0, 10, 2000, 100, 3000);

    checkMath(tmm, 0, 0, 1, 10, 2500, 55, 3000);
    checkMath(tmm, 1, 0, 1, 55, 2500, 100, 3000);
    checkMath(tmm, 0, 1, 1, 10, 2000, 55, 2500);
    checkMath(tmm, 1, 1, 1, 55, 2000, 100, 2500);

    uint32_t c, r;
    double x, y;
    for (c=0, x=10; c<4; c++, x+=22.5)
    {
        for (r=0, y=3000-250; r<4; r++, y+=-250)
        {
          checkMath(tmm, c, r, 2, x, y, x+22.5, y+250);
        }
    }

    checkChildren(tmm, 0, 0, 0,
                  0, 0, 1, 0, 0, 1, 1, 1);
    checkChildren(tmm, 1, 0, 0,
                  2, 0, 3, 0, 2, 1, 3, 1);

    checkChildren(tmm, 0, 0, 1,
                  0, 0, 1, 0, 0, 1, 1, 1);
    checkChildren(tmm, 1, 0, 1,
                  2, 0, 3, 0, 2, 1, 3, 1);
    checkChildren(tmm, 2, 0, 1,
                  4, 0, 5, 0, 4, 1, 5, 1);
    checkChildren(tmm, 3, 0, 1,
                  6, 0, 7, 0, 6, 1, 7, 1);
    checkChildren(tmm, 0, 1, 1,
                  0, 2, 1, 2, 0, 3, 1, 3);
    checkChildren(tmm, 1, 1, 1,
                  2, 2, 3, 2, 2, 3, 3, 3);
    checkChildren(tmm, 2, 1, 1,
                  4, 2, 5, 2, 4, 3, 5, 3);
    checkChildren(tmm, 3, 1, 1,
                  6, 2, 7, 2, 6, 3, 7, 3);

}

TEST(TilerTest, test_tiler_matrix_math_three)
{
    // two cols by three rows
    // (0,0) to (20,30)

    using namespace tilercommon;

    const TileMatrixMath tmm(0.0, 0.0, 20.0, 30.0, 2u, 3u);

    EXPECT_DOUBLE_EQ(tmm.minX(), 0.0);
    EXPECT_DOUBLE_EQ(tmm.minY(), 0.0);
    EXPECT_DOUBLE_EQ(tmm.maxX(), 20.0);
    EXPECT_DOUBLE_EQ(tmm.maxY(), 30.0);

    EXPECT_TRUE(tmm.matrixContains(0.0, 0.0));
    EXPECT_FALSE(tmm.matrixContains(0.0, 30.0));
    EXPECT_FALSE(tmm.matrixContains(20.0, 0.0));
    EXPECT_FALSE(tmm.matrixContains(20.0, 30.0));

    EXPECT_EQ(tmm.numColsAtLevel(0), 2u);
    EXPECT_EQ(tmm.numRowsAtLevel(0), 3u);
    EXPECT_EQ(tmm.numColsAtLevel(1), 4u);
    EXPECT_EQ(tmm.numRowsAtLevel(1), 6u);
    EXPECT_EQ(tmm.numColsAtLevel(2), 8u);
    EXPECT_EQ(tmm.numRowsAtLevel(2), 12u);

    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(0), 10.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(0), 10.0);
    EXPECT_DOUBLE_EQ(tmm.tileWidthAtLevel(1), 5.0);
    EXPECT_DOUBLE_EQ(tmm.tileHeightAtLevel(1), 5.0);

    checkMath(tmm, 0, 0, 0, 0, 20, 10, 30);
    checkMath(tmm, 1, 0, 0, 10, 20, 20, 30);
    checkMath(tmm, 0, 1, 0, 0, 10, 10, 20);
    checkMath(tmm, 1, 1, 0, 10, 10, 20, 20);
    checkMath(tmm, 0, 2, 0, 0, 0, 10, 10);
    checkMath(tmm, 1, 2, 0, 10, 0, 20, 10);

    checkMath(tmm, 1, 2, 1, 5, 15, 10, 20);
    checkMath(tmm, 3, 5, 1, 15, 0, 20, 5);
    checkMath(tmm, 0, 5, 1, 0, 0, 5, 5);
    checkMath(tmm, 3, 0, 1, 15, 25, 20, 30);

    checkChildren(tmm, 0, 2, 0,
                  0, 4, 1, 4, 0, 5, 1, 5);

}

TEST(TilerTest, test_tiler_matrix_rect_contains_rect)
{
    using namespace tilercommon;
    
    // A completely contains B
    EXPECT_TRUE(TileMatrixMath::rectContainsRect(1.0, 1.0, 2.0, 2.0, 1.4, 1.4, 1.6, 1.6));
    
    // A is completely outside B
    EXPECT_FALSE(TileMatrixMath::rectContainsRect(1.4, 1.4, 1.6, 1.6, 1.0, 1.0, 2.0, 2.0));
    
    // A completely contains B only along x-axis
    EXPECT_FALSE(TileMatrixMath::rectContainsRect(1.0, 1.4, 2.0, 1.6, 1.4, 1.0, 1.6, 2.0));

    // A completely contains B only along y-axis
    EXPECT_FALSE(TileMatrixMath::rectContainsRect(1.4, 1.0, 1.6, 2.0, 1.0, 1.4, 2.0, 1.6));
}


const struct Data {
    double x;
    double y;
    double z;
} data[8] = {
    /*0*/ { -179.0, 89.0, 0.0},
    /*1*/ { -1.0, 89.0, 11.0},
    /*2*/ { -179.0, -89.0, 22.0},
    /*3*/ { -1.0, -89.0, 33.0},
    /*4*/ { 89.0, 1.0, 44.0},
    /*5*/ { 91.0, 1.0, 55.0},
    /*6*/ { 89.0, -1.0, 66.0},
    /*7*/ { 91.0, -1.0, 77.0}
};

typedef std::map<uint32_t, PointView*> ViewsMap;


// verify point view has the correct data
static void testPoint(PointView* view, const Data& data)
{
    EXPECT_EQ(view->size(), 1u);

    const double x = view->getFieldAs<double>(Dimension::Id::X, 0);
    const double y = view->getFieldAs<double>(Dimension::Id::Y, 0);
    const double z = view->getFieldAs<double>(Dimension::Id::Z, 0);

    EXPECT_FLOAT_EQ(x, data.x);
    EXPECT_FLOAT_EQ(y, data.y);
    EXPECT_FLOAT_EQ(z, data.z);
}


// make a map from (point view id) to (point view)
static void populateMap(ViewsMap& views, PointViewSet& outputViews)
{
    for (auto iter=outputViews.begin(); iter != outputViews.end(); ++iter) {
        PointViewPtr ptr = *iter;
        PointView* p = ptr.get();
        views[p->id()] = &(*p);
    }

    testPoint(views[3], data[0]); // quick sanity check
}


// verify tile is correct, both the metadata and the point view
static void testTileDetails(uint32_t tileId, double l, double x, double y, uint32_t m, uint32_t v, ViewsMap& views)
{
    switch (tileId) {

      case 0:
        EXPECT_TRUE(l==0 && x == 0 && y == 0);
        EXPECT_TRUE(m == 15);
        EXPECT_TRUE(v == 3);
        testPoint(views[v], data[0]);
        break;

      case 1:
        EXPECT_TRUE(l==0 && x == 1 && y == 0);
        EXPECT_TRUE(m == 15);
        EXPECT_TRUE(v = 999);
        break;

      case 2:
        EXPECT_TRUE(l==1 && x == 0 && y == 0);
        EXPECT_TRUE(m == 8);
        EXPECT_TRUE(v == 4);
        testPoint(views[v], data[0]);
        break;

      case 3:
        EXPECT_TRUE(l==2 && x == 0 && y == 0);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 5);
        testPoint(views[v], data[0]);
        break;

      case 4:
        EXPECT_TRUE(l==1 && x == 1 && y == 0);
        EXPECT_TRUE(m == 4);
        EXPECT_TRUE(v == 999);
        break;

      case 5:
        EXPECT_TRUE(l==2 && x == 3 && y == 0);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 6);
        testPoint(views[v], data[1]);
        break;

      case 6:
        EXPECT_TRUE(l==1 && x == 0 && y == 1);
        EXPECT_TRUE(m == 1);
        EXPECT_TRUE(v == 999);
        break;

      case 7:
        EXPECT_TRUE(l==2 && x == 0 && y == 3);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 7);
        testPoint(views[v], data[2]);
        break;

      case 8:
        EXPECT_TRUE(l==1 && x == 1 && y == 1);
        EXPECT_TRUE(m == 2);
        EXPECT_TRUE(v == 999);
        break;

      case 9:
        EXPECT_TRUE(l==2 && x == 3 && y == 3);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 8);
        testPoint(views[v], data[3]);
        break;

      case 10:
        EXPECT_TRUE(l==1 && x == 2 && y == 0);
        EXPECT_TRUE(m == 2);
        EXPECT_TRUE(v == 9);
        testPoint(views[v], data[4]);
        break;

      case 11:
        EXPECT_TRUE(l==2 && x == 5 && y == 1);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 10);
        testPoint(views[v], data[4]);
        break;

      case 12:
        EXPECT_TRUE(l==1 && x == 3 && y == 0);
        EXPECT_TRUE(m == 1);
        EXPECT_TRUE(v == 999);
        break;

      case 13:
        EXPECT_TRUE(l==2 && x == 6 && y == 1);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 11);
        testPoint(views[v], data[5]);
        break;

      case 14:
        EXPECT_TRUE(l==1 && x == 2 && y == 1);
        EXPECT_TRUE(m == 4);
        EXPECT_TRUE(v == 999);
        break;

      case 15:
        EXPECT_TRUE(l==2 && x == 5 && y == 2);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 12);
        testPoint(views[v], data[6]);
        break;

      case 16:
        EXPECT_TRUE(l==1 && x == 3 && y == 1);
        EXPECT_TRUE(m == 8);
        EXPECT_TRUE(v == 999);
        break;

      case 17:
        EXPECT_TRUE(l==2 && x == 6 && y == 2);
        EXPECT_TRUE(m == 0);
        EXPECT_TRUE(v == 13);
        testPoint(views[v], data[7]);
        break;

      default:
        EXPECT_TRUE(false);
    }
}


// verify the tile has the right contents
static void testTile(MetadataNode tileNode, ViewsMap& views)
{
    const uint32_t tileId = boost::lexical_cast<uint32_t>(tileNode.name());

    const MetadataNode nodeL = tileNode.findChild("level");
    const MetadataNode nodeX = tileNode.findChild("tileX");
    const MetadataNode nodeY = tileNode.findChild("tileY");
    const MetadataNode nodeM = tileNode.findChild("mask");
    EXPECT_TRUE(nodeL.valid());
    EXPECT_TRUE(nodeX.valid());
    EXPECT_TRUE(nodeY.valid());
    EXPECT_TRUE(nodeM.valid());

    const uint32_t l = boost::lexical_cast<uint32_t>(nodeL.value());
    const uint32_t x = boost::lexical_cast<uint32_t>(nodeX.value());
    const uint32_t y = boost::lexical_cast<uint32_t>(nodeY.value());
    const uint32_t m = boost::lexical_cast<uint32_t>(nodeM.value());

    const MetadataNode nodeP = tileNode.findChild("pointView");
    uint32_t v = 999;
    if (nodeP.valid()) {
      v = boost::lexical_cast<uint32_t>(nodeP.value());
    }

    testTileDetails(tileId, l, x, y, m, v, views);
}


static uint32_t getMetadataU32(const MetadataNode& parent, const std::string& name) {
    const MetadataNode node = parent.findChild(name);
    if (!node.valid()) return std::numeric_limits<uint32_t>::max();
    uint32_t v = boost::lexical_cast<uint32_t>(node.value());
    return v;
}


static double getMetadataF64(const MetadataNode& parent, const std::string& name) {
    const MetadataNode node = parent.findChild(name);
    if (!node.valid()) return std::numeric_limits<double>::infinity();
    double v = boost::lexical_cast<double>(node.value());
    return v;
}


TEST(TilerTest, test_4326)
{
    // set up test data
    PointTable table;
    PointViewPtr inputView(new PointView(table));

    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    for (int i=0; i<8; i++)
    {
        inputView->setField(Dimension::Id::X, i, data[i].x);
        inputView->setField(Dimension::Id::Y, i, data[i].y);
        inputView->setField(Dimension::Id::Z, i, data[i].z);
    }


    // options
    Options readerOptions;

    Options tilerOptions;
    tilerOptions.add("maxLevel", 2);
    tilerOptions.add("numCols", 2);
    tilerOptions.add("numRows", 1);
    tilerOptions.add("minx", -180.0);
    tilerOptions.add("miny", -90.0);
    tilerOptions.add("maxx", 180.0);
    tilerOptions.add("maxy", 90.0);

    Options statsOptions;

    // stages
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);
    reader.setSpatialReference(SpatialReference("EPSG:4326"));

    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);


    // execution
    tiler.prepare(table);
    PointViewSet outputViews = tiler.execute(table);


    // prepare for testing
    const MetadataNode root = table.metadata();
    EXPECT_TRUE(root.valid());

    // sanity check the stats metadata
    {
        bool statsOkay = false;
        const MetadataNode statsNode = root.findChild("filters.stats");
        EXPECT_TRUE(statsNode.valid());
        for (auto node1 : statsNode.children("statistic"))
        {
            EXPECT_TRUE(node1.valid());
            for (auto node2 : node1.children())
            {
                EXPECT_TRUE(node2.valid());
                if (node2.name().compare("name")==0 && node2.value().compare("Z")==0) {
                    statsOkay = true;
                    MetadataNode avg = node1.findChild("average");
                    EXPECT_TRUE(avg.value().compare("38.5") == 0);
                }
            }
        }
        EXPECT_TRUE(statsOkay);
    }

    const MetadataNode tileSetNode = root.findChild("filters.tiler");
    EXPECT_TRUE(tileSetNode.valid());

    const MetadataNode headerNode = tileSetNode.findChild("header");
    EXPECT_TRUE(headerNode.valid());

    EXPECT_EQ(getMetadataU32(headerNode, "maxLevel"), 2u);
    EXPECT_EQ(getMetadataU32(headerNode, "numCols"), 2u);
    EXPECT_EQ(getMetadataU32(headerNode, "numRows"), 1u);
    EXPECT_EQ(getMetadataF64(headerNode, "minX"), -180.0);
    EXPECT_EQ(getMetadataF64(headerNode, "minY"), -90.0);
    EXPECT_EQ(getMetadataF64(headerNode, "maxX"), 180.0);
    EXPECT_EQ(getMetadataF64(headerNode, "maxY"), 90.0);

    const MetadataNode tileNodes = tileSetNode.findChild("tiles");
    EXPECT_TRUE(tileNodes.valid());

    EXPECT_EQ(outputViews.size(), 2u + 8u + 1u);

    ViewsMap viewsMap;
    populateMap(viewsMap, outputViews);

    // real testing
    for (auto tileNode: tileNodes.children())
    {
        EXPECT_TRUE(tileNode.valid());

        testTile(tileNode, viewsMap);
    }

    // finally, check the tile set's stats metadata
    {
        MetadataNode statisticsNode = tileSetNode.findChild("statistics");
        EXPECT_TRUE(statisticsNode.valid());
        EXPECT_EQ(statisticsNode.children().size(), 3u);
        MetadataNode zNode = statisticsNode.findChild("Z");
        EXPECT_TRUE(zNode.valid());
        EXPECT_EQ(getMetadataF64(zNode, "minimum"), 0.0);
        EXPECT_EQ(getMetadataF64(zNode, "mean"), 38.5);
        EXPECT_EQ(getMetadataF64(zNode, "maximum"), 77.0);
    }
}


TEST(TilerTest, test_nostats)
{
    // set up test data
    PointTable table;
    PointViewPtr inputView(new PointView(table));

    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    for (int i=0; i<8; i++)
    {
        inputView->setField(Dimension::Id::X, i, data[i].x);
        inputView->setField(Dimension::Id::Y, i, data[i].y);
        inputView->setField(Dimension::Id::Z, i, data[i].z);
    }


    // options
    Options readerOptions;

    Options tilerOptions;
    tilerOptions.add("maxLevel", 2);
    tilerOptions.add("numCols", 2);
    tilerOptions.add("numRows", 1);
    tilerOptions.add("minx", -180.0);
    tilerOptions.add("miny", -90.0);
    tilerOptions.add("maxx", 180.0);
    tilerOptions.add("maxy", 90.0);

    Options statsOptions;

    // stages
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);
    reader.setSpatialReference(SpatialReference("EPSG:4326"));

    //StatsFilter stats;
    //stats.setOptions(statsOptions);
    //stats.setInput(reader);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(reader);


    // execution
    EXPECT_NO_THROW(tiler.prepare(table));
    EXPECT_THROW(tiler.execute(table), pdal_error);
}


TEST(TilerTest, test_cartesian_3x3)
{
    // set up test data
    PointTable table;
    PointViewPtr inputView(new PointView(table));

    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    for (int i=0; i<8; i++)
    {
        inputView->setField(Dimension::Id::X, i, data[i].x);
        inputView->setField(Dimension::Id::Y, i, data[i].y);
        inputView->setField(Dimension::Id::Z, i, data[i].z);
    }

    // options
    Options readerOptions;

    Options tilerOptions;
    tilerOptions.add("maxLevel", 0);
    tilerOptions.add("numCols", 3);
    tilerOptions.add("numRows", 3);
    tilerOptions.add("minx", -200.0);
    tilerOptions.add("miny", -100.0);
    tilerOptions.add("maxx", 200.0);
    tilerOptions.add("maxy", 100.0);

    Options statsOptions;

    // stages
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);

    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);

    // execution
    tiler.prepare(table);
    PointViewSet outputViews = tiler.execute(table);

    // prepare for testing
    const MetadataNode root = table.metadata();
    EXPECT_TRUE(root.valid());

    const MetadataNode tileSetNode = root.findChild("filters.tiler");
    EXPECT_TRUE(tileSetNode.valid());

    const MetadataNode headerNode = tileSetNode.findChild("header");
    EXPECT_TRUE(headerNode.valid());

    EXPECT_EQ(getMetadataU32(headerNode, "maxLevel"), 0u);
    EXPECT_EQ(getMetadataU32(headerNode, "numCols"), 3u);
    EXPECT_EQ(getMetadataU32(headerNode, "numRows"), 3u);
    EXPECT_EQ(getMetadataF64(headerNode, "minX"), -200.0);
    EXPECT_EQ(getMetadataF64(headerNode, "minY"), -100.0);
    EXPECT_EQ(getMetadataF64(headerNode, "maxX"), 200.0);
    EXPECT_EQ(getMetadataF64(headerNode, "maxY"), 100.0);

    const MetadataNode tileNodes = tileSetNode.findChild("tiles");
    EXPECT_TRUE(tileNodes.valid());

    EXPECT_EQ(5u, outputViews.size());
    
    auto iter = outputViews.begin();
    EXPECT_EQ(1u, (*iter)->size()); // 0
    ++iter;
    EXPECT_EQ(1u, (*iter)->size()); // 1
    ++iter;
    EXPECT_EQ(1u, (*iter)->size()); // 2
    ++iter;
    EXPECT_EQ(1u, (*iter)->size()); // 3
    ++iter;
    EXPECT_EQ(4u, (*iter)->size()); // 4
    ++iter;
    EXPECT_EQ(outputViews.end(), iter);
}


TEST(TilerTest, test_cartesian_10x20)
{
    // set up test data
    PointTable table;
    PointViewPtr inputView(new PointView(table));

    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    for (int i=0; i<8; i++)
    {
        inputView->setField(Dimension::Id::X, i, data[i].x);
        inputView->setField(Dimension::Id::Y, i, data[i].y);
        inputView->setField(Dimension::Id::Z, i, data[i].z);
    }

    // options
    Options readerOptions;

    Options tilerOptions;
    tilerOptions.add("maxLevel", 0);
    tilerOptions.add("numCols", 10);
    tilerOptions.add("numRows", 20);
    tilerOptions.add("minx", -200.0);
    tilerOptions.add("miny", -100.0);
    tilerOptions.add("maxx", 200.0);
    tilerOptions.add("maxy", 100.0);

    Options statsOptions;

    // stages
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);

    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);

    // execution
    tiler.prepare(table);
    PointViewSet outputViews = tiler.execute(table);

    // prepare for testing
    const MetadataNode root = table.metadata();
    EXPECT_TRUE(root.valid());

    const MetadataNode tileSetNode = root.findChild("filters.tiler");
    EXPECT_TRUE(tileSetNode.valid());

    const MetadataNode headerNode = tileSetNode.findChild("header");
    EXPECT_TRUE(headerNode.valid());

    EXPECT_EQ(getMetadataU32(headerNode, "maxLevel"), 0u);
    EXPECT_EQ(getMetadataU32(headerNode, "numCols"), 10u);
    EXPECT_EQ(getMetadataU32(headerNode, "numRows"), 20u);
    EXPECT_EQ(getMetadataF64(headerNode, "minX"), -200.0);
    EXPECT_EQ(getMetadataF64(headerNode, "minY"), -100.0);
    EXPECT_EQ(getMetadataF64(headerNode, "maxX"), 200.0);
    EXPECT_EQ(getMetadataF64(headerNode, "maxY"), 100.0);

    const MetadataNode tileNodes = tileSetNode.findChild("tiles");
    EXPECT_TRUE(tileNodes.valid());

    EXPECT_EQ(6u, outputViews.size());
    
    auto iter = outputViews.begin();
    EXPECT_EQ(1u, (*iter)->size()); // 0
    ++iter;
    EXPECT_EQ(1u, (*iter)->size()); // 1
    ++iter;
    EXPECT_EQ(1u, (*iter)->size()); // 2
    ++iter;
    EXPECT_EQ(1u, (*iter)->size()); // 3
    ++iter;
    EXPECT_EQ(2u, (*iter)->size()); // 4
    ++iter;
    EXPECT_EQ(2u, (*iter)->size()); // 5
    ++iter;
    EXPECT_EQ(outputViews.end(), iter);
}

TEST(TilerTest, test_boundary_bug)
{
   const double minx = -179.9;
   const double miny = -89.9;
   const double maxx = 179.9;
   const double maxy = 89.9;

   const tilercommon::TileMatrixMath tmm(-180.0, -90.0, 180.0, 90.0, 2, 1);
   uint32_t mincol, minrow, maxcol, maxrow;

   {
       tmm.getTileOfPoint(minx, miny, 0, mincol, minrow);
       tmm.getTileOfPoint(maxx, maxy, 0, maxcol, maxrow);

       EXPECT_EQ(0u, mincol);
       EXPECT_EQ(0u, minrow);
       EXPECT_EQ(1u, maxcol);
       EXPECT_EQ(0u, maxrow);
   }

   {
       tmm.getTileOfPoint(minx, miny, 1, mincol, minrow);
       tmm.getTileOfPoint(maxx, maxy, 1, maxcol, maxrow);

       EXPECT_EQ(0u, mincol);
       EXPECT_EQ(1u, minrow);
       EXPECT_EQ(3u, maxcol);
       EXPECT_EQ(0u, maxrow);
   }

   {
       tmm.getTileOfPoint(minx, miny, 2, mincol, minrow);
       tmm.getTileOfPoint(maxx, maxy, 2, maxcol, maxrow);

       EXPECT_EQ(0u, mincol);
       EXPECT_EQ(3u, minrow);
       EXPECT_EQ(7u, maxcol);
       EXPECT_EQ(0u, maxrow);
   }
}
