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
#include "RialtoTest.hpp"

using namespace pdal;


static bool testP2T(double x, double y, uint32_t level, uint32_t expected_col, uint32_t expected_row)
{
    uint32_t actual_col, actual_row;
    RialtoDb::xyPointToTileColRow(x, y, level, actual_col, actual_row); // nw
    bool r = actual_col == expected_col;
    bool c = actual_row == expected_row;
    return r && c;
}


void verifyDatabase(const std::string& filename, RialtoTest::Data* actualData)
{
    LogPtr log(new Log("rialtodbwritertest", "stdout"));
    
    RialtoDb db(filename, log);
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
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[0]);
    }

    {
        // TODO: these two are order-dependent
        db.readTileInfo(tilesAt1[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[0]);

        db.readTileInfo(tilesAt1[1], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[4]);
    }

    {
        // TODO: these eight are order-dependent
        db.readTileInfo(tilesAt2[0], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[0]);

        db.readTileInfo(tilesAt2[1], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[1]);

        db.readTileInfo(tilesAt2[2], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[2]);

        db.readTileInfo(tilesAt2[3], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[3]);

        db.readTileInfo(tilesAt2[4], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[4]);

        db.readTileInfo(tilesAt2[5], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[5]);

        db.readTileInfo(tilesAt2[6], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[6]);

        db.readTileInfo(tilesAt2[7], true, info);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        RialtoTest::verifyPointFromBuffer(info.patch.buf, actualData[7]);
    }
}

TEST(RialtoDbWriterTest, testPointToTile)
{
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
{
    const std::string filename = Support::temppath("./test1.sqlite");

    FileUtils::deleteFile(filename);

    LogPtr log(new Log("rialtodbwritertest", "stdout"));

    {
        RialtoDb db(filename, log);
        db.create();
        db.close();
    }
    
    EXPECT_TRUE(FileUtils::fileExists(filename));
    
    {
        RialtoDb db(filename, log);
        db.open(true);
    }
    
    {
        RialtoDb db(filename, log);
        db.open(false);
        db.close();
    }

    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, createWriter)
{
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialtodb"));
    EXPECT_TRUE(writer.get());
}


TEST(RialtoDbWriterTest, testWriter)
{
    const std::string filename(Support::temppath("rialto2.sqlite"));
    
    FileUtils::deleteFile(filename);

    // set up test data
    PointTable table;
    PointViewPtr inputView(new PointView(table));
    RialtoTest::Data* actualData = RialtoTest::sampleDataInit(table, inputView);

    // stages
    RialtoTest::createDatabase(table, inputView, filename, 2);
    
    verifyDatabase(filename, actualData);
    
    // verification
    /*for (int i=0; i<8; i++) {
        db.readTileInfo(tilesAt2[i], true, info);
        //printf("id: %d\n", tilesAt2[i]);
        //printf(" column: %d\n", info.column);
        //printf(" row: %d\n", info.row);
        //printf(" level: %d\n", info.level);
    }*/

    LogPtr log(new Log("rialtodbwritertest", "stdout"));

    RialtoDb db(filename, log);
    db.open(false);
    std::vector<uint32_t> tileSetIds;
    db.readTileSetIds(tileSetIds);
    uint32_t tileSetId = tileSetIds[0];

    {
        std::vector<uint32_t> ids;

        db.queryForTileIds(tileSetId, 0.1, 0.1, 179.9, 89.9, 0, ids);
        EXPECT_EQ(ids.size(), 0u);

        db.queryForTileIds(tileSetId, 0.1, 0.1, 179.9, 89.9, 1, ids);
        EXPECT_EQ(ids.size(), 1u);
        EXPECT_EQ(ids[0], 7u);

        db.queryForTileIds(tileSetId, 0.1, 0.1, 179.9, 89.9, 2, ids);
        EXPECT_EQ(ids.size(), 2u);
        EXPECT_EQ(ids[0], 8u);
        EXPECT_EQ(ids[1], 9u);
    }

    {
        PointTable table;
        db.setupPointTable(tileSetId, table);

        Stage* stage = db.query(table, tileSetIds[0], 0.1, 0.1, 179.9, 89.9, 2);

        // execution
        stage->prepare(table);
        PointViewSet outputViews = stage->execute(table);

        EXPECT_EQ(outputViews.size(), 1u);
        PointViewPtr outView = *(outputViews.begin());
        EXPECT_EQ(outView->size(), 2u);
        RialtoTest::verifyPointToData(outView, 0, actualData[4]);
        RialtoTest::verifyPointToData(outView, 1, actualData[5]);
    }

    db.close();

    delete[] actualData;
    
    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, testOscar)
{
    const std::string filename(Support::temppath("oscar.sqlite"));
    
    FileUtils::deleteFile(filename);

    RialtoTest::Data* actualData;

    // make a test database
    {
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        actualData = RialtoTest::sampleDataInit(table, inputView);

        // stages
        RialtoTest::createDatabase(table, inputView, filename, 2);
    }

    // now read from it
    {
        LogPtr log(new Log("rialtodbwritertest", "stdout"));

        // open the db for reading
        RialtoDb db(filename, log);
        db.open(false);

        // we only have 1 tile set in the database: get it's id
        std::vector<uint32_t> tileSetIds;
        db.readTileSetIds(tileSetIds);
        const uint32_t tileSetId = tileSetIds[0];

        // how many levels do we have?
        RialtoDb::TileSetInfo tileSetInfo;
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
        RialtoTest::verifyPointToData(view, 0, actualData[4]);

        // That was so much fun, let's do it again!
        Stage* stage2 = db.query(table, tileSetId, -179.9, -89.9, -0.1, -0.1, bestLevel);
        stage2->prepare(table);
        views = stage2->execute(table);

        // check output again
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 2u);
        RialtoTest::verifyPointToData(view, 0, actualData[2]);
        RialtoTest::verifyPointToData(view, 1, actualData[3]);

        // And a third time!
        Stage* stage3 = db.query(table, tileSetId, 50.0, 50.0, 51.0, 51.0, bestLevel);
        stage3->prepare(table);
        views = stage3->execute(table);
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 0u);

        db.close();
    }

    delete[] actualData;

    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, testLarge)
{
    static const int NUM_POINTS = 1000;
    static const int NUM_QUERIES = 1000;

    const std::string filename(Support::temppath("rialto3.sqlite"));
    FileUtils::deleteFile(filename);

    RialtoTest::Data* actualData;

    const uint32_t maxLevel = 3;
    
    // make a test database
    {
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        actualData = RialtoTest::randomDataInit(table, inputView, NUM_POINTS);

        RialtoTest::createDatabase(table, inputView, filename, maxLevel);
    }

    // now read from it
    {
        LogPtr log(new Log("rialtodbwritertest", "stdout"));
        
        // open the db for reading
        RialtoDb db(filename, log);
        db.open(false);

        // we only have 1 tile set in the database: get it's id
        std::vector<uint32_t> tileSetIds;
        db.readTileSetIds(tileSetIds);
        const uint32_t tileSetId = tileSetIds[0];

        RialtoDb::TileSetInfo tileSetInfo;
        db.readTileSetInfo(tileSetId, tileSetInfo);
        const uint32_t bestLevel = tileSetInfo.maxLevel;

        PointTable table;
        db.setupPointTable(tileSetId, table);
        PointViewSet views;
        PointViewPtr view;

        for (int i=0; i<NUM_QUERIES; i++)
        {
            double minx = Utils::random(-179.9, 179.9);
            double maxx = Utils::random(-179.9, 179.9);
            if (minx > maxx) std::swap(minx, maxx);
            double miny = Utils::random(-89.9, 89.9);
            double maxy = Utils::random(-89.9, 89.9);
            if (miny > maxy) std::swap(miny, maxy);

            Stage* stage1 = db.query(table, tileSetId, minx, miny, maxx, maxy, bestLevel);

            stage1->prepare(table);
            views = stage1->execute(table);
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            uint32_t c = view->size();

            RialtoTest::verifyPointsInBounds(view, minx, miny, maxx, maxy);
            uint32_t expected = RialtoTest::countPointsInBounds(actualData, NUM_POINTS, minx, miny, maxx, maxy);
            EXPECT_EQ(expected, view->size());
        }
    }
    
    delete[] actualData;
    
    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, testPerf)
{
    return;
    
    static const int NUM_POINTS = 50000;
    static const int NUM_QUERIES = 50;
    
    Utils::random_seed(17);
    
    const std::string filename(Support::temppath("perf.sqlite"));
    FileUtils::deleteFile(filename);

    const uint32_t maxLevel = 6;
    
    RialtoTest::Data* actualData;

    // make a test database
    /***/ double create_ms = 0.0;
    for (int i=0; i<3; i++)
    {
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        actualData = RialtoTest::randomDataInit(table, inputView, NUM_POINTS);

        /***/clock_t start = RialtoDb::timerStart();
        RialtoTest::createDatabase(table, inputView, filename, maxLevel);
        /***/double ms = RialtoDb::timerStop(start);
        /***/printf("CREATE: %f\n", ms);
        create_ms += ms;
    }
    /***/printf("CREATE avg: %f\n", create_ms/3.0);

    // now read from it
    {
        LogPtr log(new Log("rialtodbwritertest", "stdout"));
        
        // open the db for reading
        RialtoDb db(filename, log);
        db.open(false);

        // we only have 1 tile set in the database: get it's id
        std::vector<uint32_t> tileSetIds;
        db.readTileSetIds(tileSetIds);
        const uint32_t tileSetId = tileSetIds[0];

        RialtoDb::TileSetInfo tileSetInfo;
        db.readTileSetInfo(tileSetId, tileSetInfo);
        const uint32_t bestLevel = tileSetInfo.maxLevel;

        PointTable table;
        db.setupPointTable(tileSetId, table);
        PointViewSet views;
        PointViewPtr view;

        /***/ double query_ms = 0.0;
        
        for (int i=0; i<NUM_QUERIES; i++)
        {
            double minx = Utils::random(-179.9, 179.9);
            double maxx = Utils::random(-179.9, 179.9);
            if (minx > maxx) std::swap(minx, maxx);
            double miny = Utils::random(-89.9, 89.9);
            double maxy = Utils::random(-89.9, 89.9);
            if (miny > maxy) std::swap(miny, maxy);

            /***/clock_t start = RialtoDb::timerStart();
            
            Stage* stage1 = db.query(table, tileSetId, minx, miny, maxx, maxy, bestLevel);

            stage1->prepare(table);
            views = stage1->execute(table);
            
            /***/double ms = RialtoDb::timerStop(start);
            /***/printf("QUERY: %f\n", ms);
            /***/ query_ms += ms;
            
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            uint32_t c = view->size();

            RialtoTest::verifyPointsInBounds(view, minx, miny, maxx, maxy);
            uint32_t expected = RialtoTest::countPointsInBounds(actualData, NUM_POINTS, minx, miny, maxx, maxy);
            EXPECT_EQ(expected, view->size());
        }
        /***/printf("QUERY tot: %f\n", query_ms);
        /***/printf("QUERY avg: %f\n", query_ms/(double)NUM_QUERIES);
    }
    
    delete[] actualData;
    
    FileUtils::deleteFile(filename);
}
