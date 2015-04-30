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

#include "Support.hpp"

using namespace pdal;


const struct {
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


static void testPoint(const PointView& view, uint32_t idx)
{
    EXPECT_EQ(view.size(), 1u);
    
    const double x = view.getFieldAs<double>(Dimension::Id::X, 0);
    const double y = view.getFieldAs<double>(Dimension::Id::Y, 0);
    const double z = view.getFieldAs<double>(Dimension::Id::Z, 0);

    EXPECT_FLOAT_EQ(x, data[idx].x);
    EXPECT_FLOAT_EQ(y, data[idx].y);
    EXPECT_FLOAT_EQ(z, data[idx].z);
}


TEST(TilerTest, test_tiler_filter)
{
    // test data
    PointTable inputTable;
    PointViewPtr inputView(new PointView(inputTable));

    inputTable.layout()->registerDim(Dimension::Id::X);
    inputTable.layout()->registerDim(Dimension::Id::Y);
    inputTable.layout()->registerDim(Dimension::Id::Z);

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


    // stages
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(reader);


    // execution
    PointTable outputTable;
    tiler.prepare(outputTable);

    PointViewSet outputViews = tiler.execute(outputTable);

    
    // testing
    EXPECT_EQ(outputViews.size(), 2u + 8u + 1u);

    PointViewPtr tmp = *outputViews.begin();
    const MetadataNode rootNode = tmp->metadata().findChild("tiles");
    EXPECT_TRUE(rootNode.valid());
    MetadataNodeList children = rootNode.children();
    EXPECT_TRUE(children.size() == outputViews.size());
        
    for (auto iter = outputViews.begin(); iter != outputViews.end(); ++iter)
    {
        PointViewPtr outputView = *iter;
    
        const std::string idString = std::to_string(outputView->id());
        const uint32_t id = boost::lexical_cast<uint32_t>(idString);

        const MetadataNode node = rootNode.findChild(idString);
        EXPECT_TRUE(node.valid());
        
        const MetadataNode nodeL = node.findChild("level");
        const MetadataNode nodeX = node.findChild("tileX");
        const MetadataNode nodeY = node.findChild("tileY");
        EXPECT_TRUE(nodeL.valid());
        EXPECT_TRUE(nodeX.valid());
        EXPECT_TRUE(nodeY.valid());

        const uint32_t l = boost::lexical_cast<uint32_t>(nodeL.value());
        const uint32_t x = boost::lexical_cast<uint32_t>(nodeX.value());
        const uint32_t y = boost::lexical_cast<uint32_t>(nodeY.value());

        //if (!(l == 2 && x == 0 && y == 0)) continue;
        
        uint32_t idx = 3141579;
        if (l == 0 && x == 0 && y == 0) {
            idx = 0;
        }
        
        else if (l == 1 && x == 0 && y == 0) {
            idx = 0;
        } else if (l == 1 && x == 2 && y == 0) {
            idx = 4;
        }
        
        else if (l == 2 && x == 0 && y == 0) {
            idx = 0;
        } else if (l == 2 && x == 3 && y == 0) {
            idx = 1;
        } else if (l == 2 && x == 0 && y == 3) {
            idx = 2;
        } else if (l == 2 && x == 3 && y == 3) {
            idx = 3;
        } else if (l == 2 && x == 5 && y == 1) {
            idx = 4;
        } else if (l == 2 && x == 6 && y == 1) {
            idx = 5;
        } else if (l == 2 && x == 5 && y == 2) {
            idx = 6;
        } else if (l == 2 && x == 6 && y == 2) {
            idx = 7;
        } else {
            //printf("%s: %d %d %d\n", idString.c_str(), l, x, y);
            EXPECT_TRUE(false);
        }
        
        EXPECT_TRUE(idx < 8);
        
        //printf("%s:   %d %d %d == %d\n", idString.c_str(), l, x, y, idx);
        testPoint(*outputView, idx);
    }
}
