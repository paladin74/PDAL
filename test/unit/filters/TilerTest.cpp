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


static void getPoint(const PointView& data, double& x, double& y, double& z)
{
    x = data.getFieldAs<double>(Dimension::Id::X, 0);
    y = data.getFieldAs<double>(Dimension::Id::Y, 0);
    z = data.getFieldAs<double>(Dimension::Id::Z, 0);
}


TEST(TilerTest, test_tiler_filter)
{
    // test data
    PointTable inputTable;
    PointViewPtr inputView(new PointView(inputTable));

    inputTable.layout()->registerDim(Dimension::Id::X);
    inputTable.layout()->registerDim(Dimension::Id::Y);
    inputTable.layout()->registerDim(Dimension::Id::Z);

    {
        inputView->setField(Dimension::Id::X, 0, -179.0);
        inputView->setField(Dimension::Id::Y, 0, 89.0);
        inputView->setField(Dimension::Id::Z, 0, 0.0);
        
        inputView->setField(Dimension::Id::X, 1, -1.0);
        inputView->setField(Dimension::Id::Y, 1, 89.0);
        inputView->setField(Dimension::Id::Z, 1, 11.0);

        inputView->setField(Dimension::Id::X, 2, -179.0);
        inputView->setField(Dimension::Id::Y, 2, -89.0);
        inputView->setField(Dimension::Id::Z, 2, 22.0);

        inputView->setField(Dimension::Id::X, 3, -1.0);
        inputView->setField(Dimension::Id::Y, 3, -89.0);
        inputView->setField(Dimension::Id::Z, 3, 33.0);

        inputView->setField(Dimension::Id::X, 4, 89.0);
        inputView->setField(Dimension::Id::Y, 4, 1.0);
        inputView->setField(Dimension::Id::Z, 4, 44.0);
        
        inputView->setField(Dimension::Id::X, 5, 91.0);
        inputView->setField(Dimension::Id::Y, 5, 1.0);
        inputView->setField(Dimension::Id::Z, 5, 55.0);

        inputView->setField(Dimension::Id::X, 6, 89.0);
        inputView->setField(Dimension::Id::Y, 6, -1.0);
        inputView->setField(Dimension::Id::Z, 6, 66.0);

        inputView->setField(Dimension::Id::X, 7, 91.0);
        inputView->setField(Dimension::Id::Y, 7, -1.0);
        inputView->setField(Dimension::Id::Z, 7, 77.0);
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
    PointViewPtr outputView = *outputViews.begin();

    double x, y, z;
    getPoint(*outputView.get(), x, y, z);

    const double postX = -93.351563;
    const double postY = 41.577148;
    const double postZ = 16.000000;

    //EXPECT_FLOAT_EQ(x, postX);
    //EXPECT_FLOAT_EQ(y, postY);
    //EXPECT_FLOAT_EQ(z, postZ);
}
