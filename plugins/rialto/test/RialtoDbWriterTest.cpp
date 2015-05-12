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

#include "../plugins/rialto/io/RialtoDb.hpp" // TODO: fix path
#include "../plugins/sqlite/io/SQLiteCommon.hpp" // TODO: fix path

using namespace pdal;

static bool testP2T(double x, double y, uint32_t level, uint32_t expected_col, uint32_t expected_row)
{
    using namespace rialtosupport;

    uint32_t actual_col, actual_row;
    RialtoDb::xyPointToTileColRow(x, y, level, actual_col, actual_row); // nw
    bool r = actual_col == expected_col;
    bool c = actual_row == expected_row;
    return r && c;
}


TEST(RialtoDbWriterTest, testPointToTile)
{
    using namespace rialtosupport;

    uint32_t c, r;

    // level 0, four corners
    EXPECT_TRUE(testP2T(-180.0, 90.0, 0, 0, 0)); // nw
    EXPECT_TRUE(testP2T(-179, 89, 0, 0, 0)); // nw

    EXPECT_TRUE(testP2T(180, 90, 0, 0, 0)); // ne (180 wraps to -180)
    EXPECT_TRUE(testP2T(179, 89, 0, 1, 0)); // ne

    EXPECT_TRUE(testP2T(-180, -90, 0, 0, 0)); // sw
    EXPECT_TRUE(testP2T(-179, -89, 0, 0, 0)); // sw

    EXPECT_TRUE(testP2T(180, -90, 0, 0, 0)); // se (180 wraps to -180)
    EXPECT_TRUE(testP2T(179, -89, 0, 1, 0)); // se

    EXPECT_TRUE(testP2T(0, 0, 0, 1, 0)); // center
    EXPECT_TRUE(testP2T(-1, 1, 0, 0, 0)); // center nw
    EXPECT_TRUE(testP2T(1, 1, 0, 1, 0)); // center ne
    EXPECT_TRUE(testP2T(-1, -1, 0, 0, 0)); // center sw
    EXPECT_TRUE(testP2T(1, -1, 0, 1, 0)); // center se

    // level 1

    EXPECT_TRUE(testP2T(-179, 89, 1, 0, 0));

    EXPECT_TRUE(testP2T(89, 1, 1, 2, 0));

    // level 2

    EXPECT_TRUE(testP2T(-179, 89, 2, 0, 0));

    EXPECT_TRUE(testP2T(-1, 89, 2, 3, 0));

    EXPECT_TRUE(testP2T(91, -1, 2, 6, 2));
}


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
{
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


static void verifyBytes(std::vector<unsigned char>& buf,
                        const Data* expectedData)
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

    EXPECT_EQ(u.s.x, expectedData->x);
    EXPECT_EQ(u.s.y, expectedData->y);
    EXPECT_EQ(u.s.z, expectedData->z);
}


// verify point view has the correct data
static void testPoint(PointViewPtr view, PointId idx, const Data& data)
{
    const double x = view->getFieldAs<double>(Dimension::Id::X, idx);
    const double y = view->getFieldAs<double>(Dimension::Id::Y, idx);
    const double z = view->getFieldAs<double>(Dimension::Id::Z, idx);

    EXPECT_FLOAT_EQ(x, data.x);
    EXPECT_FLOAT_EQ(y, data.y);
    EXPECT_FLOAT_EQ(z, data.z);
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

    Options writerOptions;
    writerOptions.add("filename", Support::temppath("rialto2.sqlite"));
    writerOptions.add("overwrite", true);
    //writerOptions.add("verbose", LogLevel::Debug);
    StageFactory f;
    Stage* writer = f.createStage("writers.rialtodb");
    writer->setOptions(writerOptions);
    writer->setInput(tiler);

    // execution: write to database
    writer->prepare(table);
    PointViewSet outputViews = writer->execute(table);
    delete writer;

    // verification
    rialtosupport::RialtoDb db(Support::temppath("rialto2.sqlite"));
    db.open(false);

    std::vector<uint32_t> tileSetIds;
    db.readTileSetIds(tileSetIds);
    EXPECT_EQ(tileSetIds.size(), 1u);

    rialtosupport::RialtoDb::TileSetInfo tileSetInfo;
    db.readTileSetInfo(tileSetIds[0], tileSetInfo);
    EXPECT_EQ(tileSetInfo.maxLevel, 2u);
    EXPECT_EQ(tileSetInfo.numDimensions, 3u);

    const std::vector<rialtosupport::RialtoDb::DimensionInfo>& dimensionsInfo = tileSetInfo.dimensions;
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

    rialtosupport::RialtoDb::TileInfo info;

    {
        db.readTileInfo(tilesAt0[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[0]);
    }

    {
        // TODO: these two are order-dependent
        db.readTileInfo(tilesAt1[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[0]);

        db.readTileInfo(tilesAt1[1], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[4]);
    }

    {
        // TODO: these eight are order-dependent
        db.readTileInfo(tilesAt2[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[0]);

        db.readTileInfo(tilesAt2[1], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[1]);

        db.readTileInfo(tilesAt2[2], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[2]);

        db.readTileInfo(tilesAt2[3], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[3]);

        db.readTileInfo(tilesAt2[4], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[4]);

        db.readTileInfo(tilesAt2[5], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[5]);

        db.readTileInfo(tilesAt2[6], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[6]);

        db.readTileInfo(tilesAt2[7], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[7]);
    }

    /*for (int i=0; i<8; i++) {
        db.readTileInfo(tilesAt2[i], true, info);
        //printf("id: %d\n", tilesAt2[i]);
        //printf(" column: %d\n", info.column);
        //printf(" row: %d\n", info.row);
        //printf(" level: %d\n", info.level);
    }*/

    {
        std::vector<uint32_t> ids;

        db.queryForTileIds(tileSetIds[0], 0.1, 0.1, 179.9, 89.9, 0, ids);
        EXPECT_EQ(ids.size(), 0u);

        db.queryForTileIds(tileSetIds[0], 0.1, 0.1, 179.9, 89.9, 1, ids);
        EXPECT_EQ(ids.size(), 1u);
        EXPECT_EQ(ids[0], 7u);

        db.queryForTileIds(tileSetIds[0], 0.1, 0.1, 179.9, 89.9, 2, ids);
        EXPECT_EQ(ids.size(), 2u);
        EXPECT_EQ(ids[0], 8u);
        EXPECT_EQ(ids[1], 9u);
    }

    {
        PointTable table;
        db.setupPointTable(tileSetIds[0], table);

        Stage* stage = db.query(table, tileSetIds[0], 0.1, 0.1, 179.9, 89.9, 2);

        // execution
        stage->prepare(table);
        PointViewSet outputViews = stage->execute(table);

        EXPECT_EQ(outputViews.size(), 1u);
        PointViewPtr outView = *(outputViews.begin());
        EXPECT_EQ(outView->size(), 2u);
        testPoint(outView, 0, data[4]);
        testPoint(outView, 1, data[5]);
    }

    db.close();

    //FileUtils::deleteDirectory(Support::temppath("rialto2.sqlite"));
}


TEST(RialtoDbWriterTest, testOscar)
{
    const std::string filename(Support::temppath("oscar.sqlite"));
    FileUtils::deleteFile(filename);

    // make a test database
    {
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

        Options writerOptions;
        writerOptions.add("filename", filename);
        writerOptions.add("overwrite", true);
        //writerOptions.add("verbose", LogLevel::Debug);
        StageFactory f;
        Stage* writer = f.createStage("writers.rialtodb");
        writer->setOptions(writerOptions);
        writer->setInput(tiler);

        writer->prepare(table);
        PointViewSet outputViews = writer->execute(table);
        delete writer;
    }

    // now read from it
    {
        // open the db for reading
        rialtosupport::RialtoDb db(filename);
        db.open(false);

        // we only have 1 tile set in the database: get it's id
        std::vector<uint32_t> tileSetIds;
        db.readTileSetIds(tileSetIds);
        const uint32_t tileSetId = tileSetIds[0];

        // how many levels do we have?
        rialtosupport::RialtoDb::TileSetInfo tileSetInfo;
        db.readTileSetInfo(tileSetId, tileSetInfo);
        const uint32_t bestLevel = tileSetInfo.maxLevel;

        // NOTE: this table/query/stage/execute mechanism will likely be
        // replaced by a formal Reader down the road, in proper pdal fashion

        // get ready to execute...
        PointTable table;
        db.setupPointTable(tileSetId, table);
        PointViewSet views;
        PointViewPtr view;

        // do the big expensive query, which builds your pipeline
        // NOTE: api will change, this should be a StagePtr or some such
        // NOTE: api may change, I should use the BBOX class probably
        Stage* stage1 = db.query(table, tileSetId, 0.1, 0.1, 90.0, 89.9, bestLevel);

        // NOTE: we'll always return a valid stage, even if we know there are no points
        EXPECT_TRUE(stage1 != NULL);

        // go!
        stage1->prepare(table);
        views = stage1->execute(table);

        // check our output: we should have one point view, with 2 points in it
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 1u);
        testPoint(view, 0, data[4]);

        // That was so much fun, let's do it again!
        Stage* stage2 = db.query(table, tileSetId, -179.9, -89.9, -0.1, -0.1, bestLevel);
        stage2->prepare(table);
        views = stage2->execute(table);

        // check output again
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 2u);
        testPoint(view, 0, data[2]);
        testPoint(view, 1, data[3]);

        // And a third time!
        Stage* stage3 = db.query(table, tileSetId, 50.0, 50.0, 51.0, 51.0, bestLevel);
        stage3->prepare(table);
        views = stage3->execute(table);
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 0u);

        db.close();
    }

    //FileUtils::deleteDirectory(Support::temppath("oscar.sqlite"));
}



static const int countP = 10000;
static const int countQ = 100;

static struct XYZ {
    double x;
    double y;
    double z;
} xyz[countP];
static bool expected[countP];
static bool actual[countP];

static void verify(PointViewPtr view,
                   double minx, double miny, double maxx, double maxy)
{

    assert(view->size() <= countP);

    for (int i=0; i<countP; i++) expected[i] = actual[i] = false;

    int cntE = 0;
    int cntA = 0;

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
        actual[i] = true;
        ++cntA;
    }

    for (uint32_t i=0; i<countP; i++)
    {
        if (xyz[i].x >= minx && xyz[i].x <= maxx && xyz[i].y >= miny && xyz[i].y <= maxy)
        {
            expected[i] = true;
            cntE++;
        }
    }

    EXPECT_EQ(cntE, cntA);
}


TEST(RialtoDbWriterTest, testLarge)
{
    FileUtils::deleteFile(Support::temppath("rialto3.sqlite"));

    for (int i=0; i<countP; i++)
    {
        xyz[i].x = Utils::random(-179.9, 179.9);
        xyz[i].y = Utils::random(-89.9, 89.9);
        xyz[i].z = i;
    }

    {
        // set up test data
        PointTable table;
        PointViewPtr inputView(new PointView(table));

        table.layout()->registerDim(Dimension::Id::X);
        table.layout()->registerDim(Dimension::Id::Y);
        table.layout()->registerDim(Dimension::Id::Z);

        for (int i=0; i<countP; i++)
        {
            inputView->setField(Dimension::Id::X, i, xyz[i].x);
            inputView->setField(Dimension::Id::Y, i, xyz[i].y);
            inputView->setField(Dimension::Id::Z, i, xyz[i].z);
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

        Options writerOptions;
        writerOptions.add("filename", Support::temppath("rialto3.sqlite"));
        writerOptions.add("overwrite", true);
        //writerOptions.add("verbose", LogLevel::Debug);
        StageFactory f;
        Stage* writer = f.createStage("writers.rialtodb");
        writer->setOptions(writerOptions);
        writer->setInput(tiler);

        // execution: write to database
        writer->prepare(table);
        PointViewSet outputViews = writer->execute(table);
        delete writer;
    }

    // now read from it
    {
        // open the db for reading
        rialtosupport::RialtoDb db(Support::temppath("rialto3.sqlite"));
        db.open(false);

        // we only have 1 tile set in the database: get it's id
        std::vector<uint32_t> tileSetIds;
        db.readTileSetIds(tileSetIds);
        const uint32_t tileSetId = tileSetIds[0];

        rialtosupport::RialtoDb::TileSetInfo tileSetInfo;
        db.readTileSetInfo(tileSetId, tileSetInfo);
        const uint32_t bestLevel = tileSetInfo.maxLevel;

        PointTable table;
        db.setupPointTable(tileSetId, table);
        PointViewSet views;
        PointViewPtr view;

        for (int i=0; i<countQ; i++)
        {
            double minx = 23.148880;//Utils::random(-179.9, 179.9);
            double maxx = 75.463685;//Utils::random(-179.9, 179.9);
            //if (minx > maxx) std::swap(minx, maxx);
            double miny = Utils::random(-89.9, 89.9);
            double maxy = Utils::random(-89.9, 89.9);
            if (miny > maxy) std::swap(miny, maxy);

            Stage* stage1 = db.query(table, tileSetId, minx, miny, maxx, maxy, bestLevel);

            stage1->prepare(table);
            views = stage1->execute(table);
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            uint32_t c = view->size();

            verify(view, minx, miny, maxx, maxy);
        }
    }
}
