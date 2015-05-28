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
#include <crop/CropFilter.hpp>
#include <stats/StatsFilter.hpp>

#include "Support.hpp"

#include <boost/filesystem.hpp>

#include <../plugins/rialto/io/GeoPackage.hpp>
#include <../plugins/rialto/io/GeoPackageCommon.hpp>
#include <../plugins/rialto/io/RialtoDbReader.hpp>
#include <../plugins/sqlite/io/SQLiteCommon.hpp>
#include <../filters/tiler/TilerCommon.hpp>
#include "RialtoTest.hpp"

using namespace pdal;
using namespace rialto;

static bool testP2T(double x, double y, uint32_t level, uint32_t expected_col, uint32_t expected_row)
{
    uint32_t actual_col, actual_row;

    const tilercommon::TileMatrixMath tmm(-180.0, -90.0, 180.0, 90.0, 2, 1);
    tmm.getTileOfPoint(x, y, level, actual_col, actual_row);

    const bool r = actual_col == expected_col;
    const bool c = actual_row == expected_row;
    return r && c;
}


void verifyDatabase(const std::string& filename, RialtoTest::Data* actualData)
{
    LogPtr log(new Log("rialtodbwritertest", "stdout"));

    GeoPackageReader db(filename, log);
    db.open();

    std::vector<std::string> names;
    db.readMatrixSetNames(names);
    EXPECT_EQ(names.size(), 1u);

    GpkgMatrixSet info;
    db.readMatrixSet(names[0], info);
    EXPECT_EQ(2u, info.getMaxLevel());
    EXPECT_EQ(info.getNumDimensions(), 3u);

    EXPECT_DOUBLE_EQ(info.getDataMinX(), -179.0);
    EXPECT_DOUBLE_EQ(info.getDataMinY(), -89.0);
    EXPECT_DOUBLE_EQ(info.getDataMaxX(), 91.0);
    EXPECT_DOUBLE_EQ(info.getDataMaxY(), 89.0);
    EXPECT_DOUBLE_EQ(info.getTmsetMinX(), -180.0);
    EXPECT_DOUBLE_EQ(info.getTmsetMinY(), -90.0);
    EXPECT_DOUBLE_EQ(info.getTmsetMaxX(), 180.0);
    EXPECT_DOUBLE_EQ(info.getTmsetMaxY(), 90.0);

    const std::vector<GpkgDimension>& dimensionsInfo = info.getDimensions();
    EXPECT_EQ(dimensionsInfo[0].getName(), "X");
    EXPECT_EQ(dimensionsInfo[0].getDataType(), "double");
    EXPECT_DOUBLE_EQ(dimensionsInfo[0].getMinimum(), -179.0);
    EXPECT_NEAR(dimensionsInfo[0].getMean(), 0.0, 0.00000001);
    EXPECT_DOUBLE_EQ(dimensionsInfo[0].getMaximum(), 91.0);
    EXPECT_EQ(dimensionsInfo[1].getName(), "Y");
    EXPECT_EQ(dimensionsInfo[1].getDataType(), "double");
    EXPECT_DOUBLE_EQ(dimensionsInfo[1].getMinimum(), -89.0);
    EXPECT_NEAR(dimensionsInfo[1].getMean(), 0.0, 0.00000001);
    EXPECT_DOUBLE_EQ(dimensionsInfo[1].getMaximum(), 89.0);
    EXPECT_EQ(dimensionsInfo[2].getName(), "Z");
    EXPECT_EQ(dimensionsInfo[2].getDataType(), "double");
    EXPECT_DOUBLE_EQ(dimensionsInfo[2].getMinimum(), 0.0);
    EXPECT_NEAR(dimensionsInfo[2].getMean(), 38.5, 0.00000001);
    EXPECT_DOUBLE_EQ(dimensionsInfo[2].getMaximum(), 77.0);

    std::vector<uint32_t> tilesAt0;
    db.readTileIdsAtLevel(names[0], 0, tilesAt0);
    EXPECT_EQ(tilesAt0.size(), 1u);
    std::vector<uint32_t> tilesAt1;
    db.readTileIdsAtLevel(names[0], 1, tilesAt1);
    EXPECT_EQ(tilesAt1.size(), 2u);
    std::vector<uint32_t> tilesAt2;
    db.readTileIdsAtLevel(names[0], 2, tilesAt2);
    EXPECT_EQ(tilesAt2.size(), 8u);
    std::vector<uint32_t> tilesAt3;
    db.readTileIdsAtLevel(names[0], 3, tilesAt3);
    EXPECT_EQ(tilesAt3.size(), 0u);

    GpkgTile tileInfo;

    db.readTile(names[0], tilesAt0[0], true, tileInfo);
    RialtoTest::verifyPointFromBuffer(tileInfo, actualData[0]);

    for (int i=0; i<2; i++)
    {
        db.readTile(names[0], tilesAt1[i], true, tileInfo);
        const uint32_t col = tileInfo.getColumn();
        const uint32_t row = tileInfo.getRow();
        printf("==%d  %d==\n", col, row);
        
        int idx = -1;
        if (col==0 && row==0) idx = 0;
        if (col==2 && row==0) idx = 4;
        EXPECT_NE(idx, -1);
        
        RialtoTest::verifyPointFromBuffer(tileInfo, actualData[idx]);
    }

    for (int i=0; i<8; i++)
    {
        db.readTile(names[0], tilesAt2[i], true, tileInfo);
    
        const uint32_t col = tileInfo.getColumn();
        const uint32_t row = tileInfo.getRow();
        printf("==%d  %d==\n", col, row);
        int idx = -1;
        if (col==0 && row==0) idx = 0;
        if (col==0 && row==3) idx = 2;
        if (col==3 && row==0) idx = 1;
        if (col==3 && row==3) idx = 3;
        if (col==5 && row==1) idx = 4;
        if (col==5 && row==2) idx = 6;
        if (col==6 && row==1) idx = 5;
        if (col==6 && row==2) idx = 7;
        EXPECT_NE(idx, -1);
        
        RialtoTest::verifyPointFromBuffer(tileInfo, actualData[idx]);
    }

    db.close();
}


TEST(RialtoDbWriterTest, testPointToTile)
{
    uint32_t c, r;

    // level 0, four corners
    EXPECT_TRUE(testP2T(-180.0, 89.999, 0, 0, 0)); // nw
    EXPECT_TRUE(testP2T(-179, 89, 0, 0, 0));

    EXPECT_TRUE(testP2T(179.999, 89.999, 0, 1, 0)); // ne
    EXPECT_TRUE(testP2T(179, 89, 0, 1, 0));

    EXPECT_TRUE(testP2T(-180, -90, 0, 0, 0)); // sw
    EXPECT_TRUE(testP2T(-179, -89, 0, 0, 0));

    EXPECT_TRUE(testP2T(179.999, -90, 0, 1, 0)); // se
    EXPECT_TRUE(testP2T(179, -89, 0, 1, 0));

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


TEST(RialtoDbWriterTest, createWriter)
{
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialtodb"));
    EXPECT_TRUE(writer.get());
}


TEST(RialtoDbWriterTest, testWriter)
{
    const std::string filename(Support::temppath("rialto2.gpkg"));

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

    {
        GeoPackageReader db(filename, log);
        db.open();
        std::vector<std::string> names;
        db.readMatrixSetNames(names);
        std::string tileTableName = names[0];

        {
            std::vector<uint32_t> ids;

            db.queryForTileIds(tileTableName, 0.1, 0.1, 179.9, 89.9, 0, ids);
            EXPECT_EQ(ids.size(), 0u);

            db.queryForTileIds(tileTableName, 0.1, 0.1, 179.9, 89.9, 1, ids);
            EXPECT_EQ(ids.size(), 1u);
            EXPECT_EQ(ids[0], 7u);

            db.queryForTileIds(tileTableName, 0.1, 0.1, 179.9, 89.9, 2, ids);
            EXPECT_EQ(ids.size(), 2u);
            EXPECT_EQ(ids[0], 8u);
            EXPECT_EQ(ids[1], 9u);
        }

        db.close();
    }

    {
        RialtoDbReader reader;
        Options options;
        options.add("filename", filename);
        //options.add("verbose", LogLevel::Debug);
        BOX3D bounds(0.1, 0.1, -999999, 179.9, 89.9, 999999);
        options.add("bounds", bounds);
        reader.setOptions(options);

        PointTable table;
        reader.prepare(table);
        PointViewSet viewSet = reader.execute(table);

        EXPECT_EQ(viewSet.size(), 1u);
        PointViewPtr view = *(viewSet.begin());
        EXPECT_EQ(view->size(), 2u);
        RialtoTest::verifyPointToData(view, 0, actualData[4]);
        RialtoTest::verifyPointToData(view, 1, actualData[5]);
    }

    delete[] actualData;

    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, testOscar)
{
    const std::string filename(Support::temppath("oscar.gpkg"));

    FileUtils::deleteFile(filename);

    RialtoTest::Data* actualData;

    // make a test database
    {
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        actualData = RialtoTest::sampleDataInit(table, inputView);
        RialtoTest::createDatabase(table, inputView, filename, 2);
    }

    // now read from it
    {
        LogPtr log(new Log("rialtodbwritertest", "stdout"));

        // get ready to execute...
        PointViewSet views;
        PointViewPtr view;

        // make the reader
        // it will always use the first (and only...) tile set in the db
        // the default is to use the maximum (best) level of the tile tree
        RialtoDbReader reader;
        Options options;
        options.add("filename", filename);
        //options.add("verbose", LogLevel::Debug);
        reader.setOptions(options);

        {
            BOX3D bounds1(0.1, 0.1, -999999, 89.9, 89.9, 999999);
            options.add("bounds", bounds1);
            reader.setOptions(options);

            // go!
            PointTable table1;
            reader.prepare(table1);
            views = reader.execute(table1);

            // check our output: we should have one point view, with 1 point in it
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            EXPECT_EQ(view->size(), 1u);
            RialtoTest::verifyPointToData(view, 0, actualData[4]);
        }

        // That was so much fun, let's do it again!
        {
            BOX3D bounds2(-179.9, -89.9, -999999, -0.1, -0.1, 999999);
            //Options options2;
            options.remove("bounds");
            options.add("bounds", bounds2);
            reader.setOptions(options);

            PointTable table2;
            reader.prepare(table2);
            views = reader.execute(table2);

            // check output again
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            EXPECT_EQ(view->size(), 2u);
            RialtoTest::verifyPointToData(view, 0, actualData[2]);
            RialtoTest::verifyPointToData(view, 1, actualData[3]);
        }

        // And a third time!
        {
            BOX3D bounds3(50.0, 50.0, -999999, 51.0, 51.0, 999999);
            options.remove("bounds");
            options.add("bounds", bounds3);
            reader.setOptions(options);

            PointTable table3;
            reader.prepare(table3);
            views = reader.execute(table3);
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            EXPECT_EQ(view->size(), 0u);
        }
    }

    delete[] actualData;

    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, testRandom)
{
    static const int K = 1000;
    static const int M = 1000 * K;
    static const int NUM_POINTS = 100 * K;
    static const int NUM_QUERIES = 100;

    const std::string filename(Support::temppath("rialto3.gpkg"));
    FileUtils::deleteFile(filename);

    RialtoTest::Data* actualData;

    const uint32_t maxLevel = 5;

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

        PointViewSet views;
        PointViewPtr view;

        RialtoDbReader reader;
        Options options;
        options.add("filename", filename);
        //options.add("verbose", LogLevel::Debug);
        reader.setOptions(options);

        for (int i=0; i<NUM_QUERIES; i++)
        {
            double minx = Utils::random(-179.9, 179.9);
            double maxx = Utils::random(-179.9, 179.9);
            if (minx > maxx) std::swap(minx, maxx);
            double miny = Utils::random(-89.9, 89.9);
            double maxy = Utils::random(-89.9, 89.9);
            if (miny > maxy) std::swap(miny, maxy);

            const double minz = -999999.0;
            const double maxz = 999999.0;

            BOX3D bounds(minx, miny, minz, maxx, maxy, maxz);
            options.remove("bounds");
            options.add("bounds", bounds);
            reader.setOptions(options);


            CropFilter crop;
            Options co;
            co.add("bounds", bounds);
            crop.setInput(reader);
            crop.setOptions(co);

            PointTable table;
            crop.prepare(table);
            views = crop.execute(table);
            EXPECT_EQ(views.size(), 1u);
            view = *(views.begin());
            uint32_t c = view->size();

            //RialtoTest::verifyPointsInBounds(view, minx, miny, maxx, maxy);
            uint32_t expected = RialtoTest::countPointsInBounds(actualData, NUM_POINTS, minx, miny, maxx, maxy);
            EXPECT_EQ(expected, view->size());
        }
    }

    delete[] actualData;

    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, writePerf)
{
    Event e_all("* WritePerf");
    Event e_write("* write_part");

    e_all.start();

    static const int K = 1000;
    static const int M = 1000 * K;
    static const int NUM_POINTS = 250 * K;

    const std::string filename(Support::temppath("writeperf.gpkg"));
    FileUtils::deleteFile(filename);

    RialtoTest::Data* actualData;
    const uint32_t maxLevel = 11;

    {
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        actualData = RialtoTest::randomDataInit(table, inputView, NUM_POINTS, false);

        e_write.start();
        RialtoTest::createDatabase(table, inputView, filename, maxLevel);
        e_write.stop();
    }

    delete[] actualData;

    e_all.stop();

    e_all.dump();
    e_write.dump();

    FileUtils::deleteFile(filename);
}


TEST(RialtoDbWriterTest, readPerf)
{
    Event e_all("* ReadPerf");
    Event e_read("* read_part");

    e_all.start();

    static const int K = 1000;
    static const int M = 1000 * K;
    static const int NUM_POINTS = 250 * K;
    static const int NUM_QUERIES = 25;

    const std::string filename(Support::temppath("readperf.gpkg"));
    FileUtils::deleteFile(filename);

    const uint32_t maxLevel = 3;

    {
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        RialtoTest::Data* actualData = RialtoTest::randomDataInit(table, inputView, NUM_POINTS, true);
        RialtoTest::createDatabase(table, inputView, filename, maxLevel);
        delete[] actualData;
    }

    // now read from it
    {
        LogPtr log(new Log("rialtodbwritertest", "stdout"));

        PointViewSet views;
        PointViewPtr view;

        RialtoDbReader reader;
        Options options;
        options.add("filename", filename);
        //options.add("verbose", LogLevel::Debug);
        reader.setOptions(options);

        for (int i=0; i<NUM_QUERIES; i++)
        {
            double minx = Utils::random(-179.9, 179.9);
            double maxx = Utils::random(-179.9, 179.9);
            if (minx > maxx) std::swap(minx, maxx);
            double miny = Utils::random(-89.9, 89.9);
            double maxy = Utils::random(-89.9, 89.9);
            if (miny > maxy) std::swap(miny, maxy);

            const double minz = -999999.0;
            const double maxz = 999999.0;

            e_read.start();

            BOX3D bounds(minx, miny, minz, maxx, maxy, maxz);
            options.remove("bounds");
            options.add("bounds", bounds);
            reader.setOptions(options);

            PointTable table;
            reader.prepare(table);
            views = reader.execute(table);

            e_read.stop();
        }
    }

    e_all.stop();

    e_all.dump();
    e_read.dump();

    FileUtils::deleteFile(filename);
}
