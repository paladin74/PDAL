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


TEST(TilerTest, test_tiler_filter)
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


TEST(TilerTest, test_tiler_filter_not4326)
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

    Options statsOptions;

    // stages
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);
    //reader.setSpatialReference(SpatialReference("EPSG:4326"));
    
    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);


    // execution
    EXPECT_NO_THROW(tiler.prepare(table));
    EXPECT_THROW(tiler.execute(table), pdal_error);
}


TEST(TilerTest, test_tiler_filter_nostats)
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
