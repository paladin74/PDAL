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

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include <pdal/BufferReader.hpp>
#include <tiler/TilerFilter.hpp>
#include <stats/StatsFilter.hpp>

#include "Support.hpp"

#include <boost/filesystem.hpp>

#include <pdal/../../plugins/rialto/io/RialtoDb.hpp> // TODO: fix path

using namespace pdal;


TEST(RialtoDbTest, test1)
{return;
    using namespace rialtosupport;

    const std::string connection = Support::temppath("./test1.sqlite");

    FileUtils::deleteFile(connection);

    RialtoDb db(connection);
    db.open(true);
    db.close();
}


TEST(RialtoDbWriterTest, createWriter)
{return;
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialtodb"));
    EXPECT_TRUE(writer.get());
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


static void readBytes(const std::string& filename, uint8_t* buf, uint32_t len)
{
    const char* name = Support::temppath(filename).c_str();
    FILE* fp = fopen(name, "rb");
    EXPECT_TRUE(fp != NULL);
    size_t cnt = fread(buf, 1, len, fp);
    EXPECT_EQ(cnt, len);
    fclose(fp);
}


static void verifyBytes(char* actualData,
                        const Data* expectedData)
{
    EXPECT_EQ(*(double*)(actualData+0), expectedData->x);
    EXPECT_EQ(*(double*)(actualData+8), expectedData->y);
    EXPECT_EQ(*(double*)(actualData+16), expectedData->z);
}


static void verify(const std::string& filename,
                   const Data* expectedData,
                   uint8_t expectedMask)
{
    uint32_t actualSize = FileUtils::fileSize(Support::temppath(filename));

    if (expectedData) {
        EXPECT_EQ(actualSize, 25u);
        union {
          uint8_t buf[25];
          struct {
            double x;
            double y;
            double z;
            uint8_t m;
          } actualData;
        } u;
        readBytes(filename, u.buf, 25);
        EXPECT_EQ(u.actualData.x, expectedData->x);
        EXPECT_EQ(u.actualData.y, expectedData->y);
        EXPECT_EQ(u.actualData.z, expectedData->z);
        EXPECT_EQ(u.actualData.m, expectedMask);
    } else {
        EXPECT_EQ(actualSize, 1u);
        uint8_t actualMask;
        readBytes(filename, &actualMask, 1);
        EXPECT_EQ(actualMask, expectedMask);
    }
}


static void verifyDirectorySize(const std::string& path, uint32_t expectedSize)
{
    boost::filesystem::directory_iterator iter(Support::temppath(path));
    uint32_t cnt = 0;
    while (iter != boost::filesystem::directory_iterator()) {
        ++cnt;
        ++iter;
    }
    EXPECT_EQ(cnt, expectedSize);
}


TEST(RialtoDbWriterTest, testWriter)
{
    FileUtils::deleteFile(Support::temppath("rialto2.sqlite"));

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

    // stages
    Options readerOptions;
    BufferReader reader;
    reader.setOptions(readerOptions);
    reader.addView(inputView);
    reader.setSpatialReference(SpatialReference("EPSG:4326"));

    Options statsOptions;
    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    Options tilerOptions;
    tilerOptions.add("maxLevel", 2);
    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);

    //RialtoFileWriter writer;
    Options writerOptions;
    writerOptions.add("filename", Support::temppath("rialto2.sqlite"));
    writerOptions.add("overwrite", true);
    writerOptions.add("verbose", LogLevel::Debug);
    StageFactory f;
    Stage* writer = f.createStage("writers.rialtodb");
    writer->setOptions(writerOptions);
    writer->setInput(tiler);

    // execution
    writer->prepare(table);
    PointViewSet outputViews = writer->execute(table);
    delete writer;

    // verification
    rialtosupport::RialtoDb db(Support::temppath("rialto2.sqlite"));
    db.open(false);
    
    std::vector<uint32_t> tileSetIds = db.getTileSetIds();
    EXPECT_EQ(tileSetIds.size(), 1u);
    
    rialtosupport::RialtoDb::TileSetInfo tileSetInfo = db.getTileSetInfo(tileSetIds[0]);
    EXPECT_DOUBLE_EQ(tileSetInfo.minx, -180.0);
    EXPECT_DOUBLE_EQ(tileSetInfo.miny, -90.0);
    EXPECT_DOUBLE_EQ(tileSetInfo.maxx, 180.0);
    EXPECT_DOUBLE_EQ(tileSetInfo.maxy, 90.0);
    EXPECT_EQ(tileSetInfo.maxLevel, 2u);
    EXPECT_EQ(tileSetInfo.numCols, 2u);
    EXPECT_EQ(tileSetInfo.numRows, 1u);
    EXPECT_EQ(tileSetInfo.numDimensions, 3u);

    rialtosupport::RialtoDb::DimensionInfo dimensionInfo;
    dimensionInfo = db.getDimensionInfo(tileSetIds[0], 0);
    EXPECT_EQ(dimensionInfo.name, "X");
    EXPECT_EQ(dimensionInfo.dataType, rialtosupport::RialtoDb::Float64);
    EXPECT_DOUBLE_EQ(dimensionInfo.minimum, -179.0);
    EXPECT_DOUBLE_EQ(dimensionInfo.mean+100.0, 0.0+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionInfo.maximum, 91.0);
    dimensionInfo = db.getDimensionInfo(tileSetIds[0], 1);
    EXPECT_EQ(dimensionInfo.name, "Y");
    EXPECT_EQ(dimensionInfo.dataType, rialtosupport::RialtoDb::Float64);
    EXPECT_DOUBLE_EQ(dimensionInfo.minimum, -89.0);
    EXPECT_DOUBLE_EQ(dimensionInfo.mean+100.0, 0.0+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionInfo.maximum, 89.0);
    dimensionInfo = db.getDimensionInfo(tileSetIds[0], 2);
    EXPECT_EQ(dimensionInfo.name, "Z");
    EXPECT_EQ(dimensionInfo.dataType, rialtosupport::RialtoDb::Float64);
    EXPECT_DOUBLE_EQ(dimensionInfo.minimum, 0.0);
    EXPECT_DOUBLE_EQ(dimensionInfo.mean+100.0, 38.5+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionInfo.maximum, 77.0);

    std::vector<uint32_t> tilesAt0 = db.getTileIdsAtLevel(0, 0);
    EXPECT_EQ(tilesAt0.size(), 1u);
    std::vector<uint32_t> tilesAt1 = db.getTileIdsAtLevel(0, 1);
    EXPECT_EQ(tilesAt1.size(), 2u);
    std::vector<uint32_t> tilesAt2 = db.getTileIdsAtLevel(0, 2);
    EXPECT_EQ(tilesAt2.size(), 8u);
    std::vector<uint32_t> tilesAt3 = db.getTileIdsAtLevel(0, 3);
    EXPECT_EQ(tilesAt3.size(), 0u);
    
    char* buf = NULL;
    uint32_t bufLen = 0;
    
    {
        db.getTileData(tilesAt0[0], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[0]);
    }

    { 
        // TODO: these two are order-dependent
        db.getTileData(tilesAt1[0], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[0]);

        db.getTileData(tilesAt1[1], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[4]);
    }

    { 
        // TODO: these eight are order-dependent
        db.getTileData(tilesAt2[0], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[0]);

        db.getTileData(tilesAt2[1], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[1]);

        db.getTileData(tilesAt2[2], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[2]);

        db.getTileData(tilesAt2[3], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[3]);
        
        db.getTileData(tilesAt2[4], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[4]);

        db.getTileData(tilesAt2[5], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[5]);

        db.getTileData(tilesAt2[6], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[6]);
        
        db.getTileData(tilesAt2[7], buf, bufLen);
        EXPECT_EQ(bufLen, 24u);
        EXPECT_TRUE(buf != NULL);
        verifyBytes(buf, &data[7]);
    }
    
    //FileUtils::deleteDirectory(Support::temppath("rialto2.sqlite"));
}
