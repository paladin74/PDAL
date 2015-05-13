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

#include <cstdint>
#include <string>

#include <pdal/BufferReader.hpp>
#include <tiler/TilerFilter.hpp>
#include <stats/StatsFilter.hpp>

namespace pdal
{
    
class PDAL_DLL RialtoTest
{
public:
    struct Data {
        double x;
        double y;
        double z;
    };
    
    static Data sampleData[8];
    static void sampleDataInit(pdal::PointTable&, pdal::PointViewPtr);
    
    static Data *randomData;
    static void randomDataInit(pdal::PointTable&, pdal::PointViewPtr, uint32_t numPoints);

    static void createDatabase(pdal::PointTable& table, pdal::PointViewPtr view, const std::string& filename);
    static void verifyDatabase(const std::string& filename);
    
    static void verifyPointFromBuffer(std::vector<unsigned char>& buf,
                                           const RialtoTest::Data& expectedData);
    static void verifyPointsInBounds(pdal::PointViewPtr view,
                                 double minx, double miny, double maxx, double maxy);
    static uint32_t countPointsInBounds(Data* xyz, uint32_t numPoints,
                                                    double minx, double miny, double maxx, double maxy);

};


void RialtoTest::sampleDataInit(pdal::PointTable& table, pdal::PointViewPtr view)
{
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    for (int i=0; i<8; i++)
    {
        view->setField(Dimension::Id::X, i, sampleData[i].x);
        view->setField(Dimension::Id::Y, i, sampleData[i].y);
        view->setField(Dimension::Id::Z, i, sampleData[i].z);
    }
}


void RialtoTest::randomDataInit(pdal::PointTable& table, pdal::PointViewPtr view, uint32_t numPoints)
{
    delete randomData;
    randomData = new Data[numPoints];
    
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    for (uint32_t i=0; i<numPoints; i++)
    {
        randomData[i].x = Utils::random(-179.9, 179.9);
        randomData[i].y = Utils::random(-89.9, 89.9);
        randomData[i].z = i;
    }

    for (uint32_t i=0; i<numPoints; i++)
    {
        view->setField(Dimension::Id::X, i, randomData[i].x);
        view->setField(Dimension::Id::Y, i, randomData[i].y);
        view->setField(Dimension::Id::Z, i, randomData[i].z);
    }
}


void RialtoTest::createDatabase(pdal::PointTable& table, pdal::PointViewPtr view, const std::string& filename)
{
    pdal::Options readerOptions;
    pdal::BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(view);
    reader.setSpatialReference(SpatialReference("EPSG:4326"));

    pdal::Options statsOptions;
    pdal::StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    pdal::Options tilerOptions;
    tilerOptions.add("maxLevel", 2);
    pdal::TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);

    pdal::Options writerOptions;
    writerOptions.add("filename", filename);
    writerOptions.add("overwrite", true);
    //writerOptions.add("verbose", LogLevel::Debug);
    pdal::StageFactory f;
    pdal::Stage* writer = f.createStage("writers.rialtodb");
    writer->setOptions(writerOptions);
    writer->setInput(tiler);

    // execution: write to database
    writer->prepare(table);
    PointViewSet outputViews = writer->execute(table);
    delete writer;
}


void RialtoTest::verifyDatabase(const std::string& filename)
{
    RialtoDb db(filename);
    db.open(false);

    std::vector<uint32_t> tileSetIds;
    db.readTileSetIds(tileSetIds);
    EXPECT_EQ(tileSetIds.size(), 1u);

    RialtoDb::TileSetInfo tileSetInfo;
    db.readTileSetInfo(tileSetIds[0], tileSetInfo);
    EXPECT_EQ(tileSetInfo.maxLevel, 2u);
    EXPECT_EQ(tileSetInfo.numDimensions, 3u);

    const std::vector<RialtoDb::DimensionInfo>& dimensionsInfo = tileSetInfo.dimensions;
    EXPECT_EQ(dimensionsInfo[0].name, "X");
    EXPECT_EQ(dimensionsInfo[0].dataType, "double");
    EXPECT_DOUBLE_EQ(dimensionsInfo[0].minimum, -179.0);
    EXPECT_DOUBLE_EQ(dimensionsInfo[0].mean+100.0, 0.0+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionsInfo[0].maximum, 91.0);
    EXPECT_EQ(dimensionsInfo[1].name, "Y");
    EXPECT_EQ(dimensionsInfo[1].dataType, "double");
    EXPECT_DOUBLE_EQ(dimensionsInfo[1].minimum, -89.0);
    EXPECT_DOUBLE_EQ(dimensionsInfo[1].mean+100.0, 0.0+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionsInfo[1].maximum, 89.0);
    EXPECT_EQ(dimensionsInfo[2].name, "Z");
    EXPECT_EQ(dimensionsInfo[2].dataType, "double");
    EXPECT_DOUBLE_EQ(dimensionsInfo[2].minimum, 0.0);
    EXPECT_DOUBLE_EQ(dimensionsInfo[2].mean+100.0, 38.5+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionsInfo[2].maximum, 77.0);

    std::vector<uint32_t> tilesAt0;
    db.readTileIdsAtLevel(tileSetIds[0], 0, tilesAt0);
    EXPECT_EQ(tilesAt0.size(), 1u);
    std::vector<uint32_t> tilesAt1;
    db.readTileIdsAtLevel(tileSetIds[0], 1, tilesAt1);
    EXPECT_EQ(tilesAt1.size(), 2u);
    std::vector<uint32_t> tilesAt2;
    db.readTileIdsAtLevel(tileSetIds[0], 2, tilesAt2);
    EXPECT_EQ(tilesAt2.size(), 8u);
    std::vector<uint32_t> tilesAt3;
    db.readTileIdsAtLevel(tileSetIds[0], 3, tilesAt3);
    EXPECT_EQ(tilesAt3.size(), 0u);

    RialtoDb::TileInfo info;
    
    {
        db.readTileInfo(tilesAt0[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[0]);
    }

    {
        // TODO: these two are order-dependent
        db.readTileInfo(tilesAt1[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[0]);

        db.readTileInfo(tilesAt1[1], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[4]);
    }

    {
        // TODO: these eight are order-dependent
        db.readTileInfo(tilesAt2[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[0]);

        db.readTileInfo(tilesAt2[1], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[1]);

        db.readTileInfo(tilesAt2[2], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[2]);

        db.readTileInfo(tilesAt2[3], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[3]);

        db.readTileInfo(tilesAt2[4], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[4]);

        db.readTileInfo(tilesAt2[5], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[5]);

        db.readTileInfo(tilesAt2[6], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[6]);

        db.readTileInfo(tilesAt2[7], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyPointFromBuffer(info.patch.buf, RialtoTest::sampleData[7]);
    }
}


void RialtoTest::verifyPointFromBuffer(std::vector<unsigned char>& buf,
                                       const RialtoTest::Data& expectedData)
{
    const unsigned char* p = &buf[0];

    union U {
      uint8_t buf[24];
      struct S {
        double x;
        double y;
        double z;
      } s;
    } u;

    for (int i=0; i<24; i++)
      u.buf[i] = buf[i];

    EXPECT_EQ(u.s.x, expectedData.x);
    EXPECT_EQ(u.s.y, expectedData.y);
    EXPECT_EQ(u.s.z, expectedData.z);
}


void RialtoTest::verifyPointsInBounds(PointViewPtr view,
                                      double minx, double miny, double maxx, double maxy)
{
    for (uint32_t i=0; i<view->size(); i++)
    {
        const double x = view->getFieldAs<double>(Dimension::Id::X, i);
        const double y = view->getFieldAs<double>(Dimension::Id::Y, i);

        if (!(x >= minx && x <= maxx))
        {
            printf("query x:  min=%lf  max=%lf  x=%lf\n", minx, maxx, x);
            printf("query y:  min=%lf  max=%lf  y=%lf\n", miny, maxy, y);
        }
        if (!(y >= miny && y <= maxy))
        {
            printf("query y:  min=%lf  max=%lf  y=%lf\n", miny, maxy, y);
        }
        EXPECT_TRUE(x >= minx && x <= maxx);
        EXPECT_TRUE(y >= miny && y <= maxy);
    }
}

uint32_t RialtoTest::countPointsInBounds(Data* xyz, uint32_t numPoints,
                   double minx, double miny, double maxx, double maxy)
{
    uint32_t cnt = 0;
    
    for (uint32_t i=0; i<numPoints; i++)
    {
        if (xyz[i].x >= minx && xyz[i].x <= maxx && xyz[i].y >= miny && xyz[i].y <= maxy)
        {
            cnt++;
        }
    }

    return cnt;
}

} // namespace pdal
