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
    writerOptions.add("verbose", LogLevel::Debug);
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
    EXPECT_EQ(dimensionInfo.dataType, "double");
    EXPECT_DOUBLE_EQ(dimensionInfo.minimum, -179.0);
    EXPECT_DOUBLE_EQ(dimensionInfo.mean+100.0, 0.0+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionInfo.maximum, 91.0);
    dimensionInfo = db.getDimensionInfo(tileSetIds[0], 1);
    EXPECT_EQ(dimensionInfo.name, "Y");
    EXPECT_EQ(dimensionInfo.dataType, "double");
    EXPECT_DOUBLE_EQ(dimensionInfo.minimum, -89.0);
    EXPECT_DOUBLE_EQ(dimensionInfo.mean+100.0, 0.0+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionInfo.maximum, 89.0);
    dimensionInfo = db.getDimensionInfo(tileSetIds[0], 2);
    EXPECT_EQ(dimensionInfo.name, "Z");
    EXPECT_EQ(dimensionInfo.dataType, "double");
    EXPECT_DOUBLE_EQ(dimensionInfo.minimum, 0.0);
    EXPECT_DOUBLE_EQ(dimensionInfo.mean+100.0, 38.5+100.0); // TODO
    EXPECT_DOUBLE_EQ(dimensionInfo.maximum, 77.0);

    std::vector<uint32_t> tilesAt0 = db.getTileIdsAtLevel(tileSetIds[0], 0);
    EXPECT_EQ(tilesAt0.size(), 1u);
    std::vector<uint32_t> tilesAt1 = db.getTileIdsAtLevel(tileSetIds[0], 1);
    EXPECT_EQ(tilesAt1.size(), 2u);
    std::vector<uint32_t> tilesAt2 = db.getTileIdsAtLevel(tileSetIds[0], 2);
    EXPECT_EQ(tilesAt2.size(), 8u);
    std::vector<uint32_t> tilesAt3 = db.getTileIdsAtLevel(tileSetIds[0], 3);
    EXPECT_EQ(tilesAt3.size(), 0u);

    rialtosupport::RialtoDb::TileInfo info;

    {
        info = db.getTileInfo(tilesAt0[0], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[0]);
    }

    {
        // TODO: these two are order-dependent
        info = db.getTileInfo(tilesAt1[0], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[0]);

        info = db.getTileInfo(tilesAt1[1], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[4]);
    }

    {
        // TODO: these eight are order-dependent
        info = db.getTileInfo(tilesAt2[0], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[0]);

        info = db.getTileInfo(tilesAt2[1], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[1]);

        info = db.getTileInfo(tilesAt2[2], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[2]);

        info = db.getTileInfo(tilesAt2[3], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[3]);

        info = db.getTileInfo(tilesAt2[4], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[4]);

        info = db.getTileInfo(tilesAt2[5], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[5]);

        info = db.getTileInfo(tilesAt2[6], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[6]);

        info = db.getTileInfo(tilesAt2[7], true);
        EXPECT_EQ(info.numPoints, 1u);
        EXPECT_EQ(info.patch.buf.size(), 24u);
        verifyBytes(info.patch.buf, &data[7]);
    }

    {
        std::vector<uint32_t> ids;

        ids = db.queryForTileIds(tileSetIds[0], 0.0, 0.0, 180.0, 90.0, 0);
        EXPECT_EQ(ids.size(), 1u); // TODO: will be 0 when turn on spatialite

        ids = db.queryForTileIds(tileSetIds[0], 0.0, 0.0, 180.0, 90.0, 1);
        EXPECT_EQ(ids.size(), 2u); // TODO: will be 1 when turn on spatialite

        ids = db.queryForTileIds(tileSetIds[0], 0.0, 0.0, 180.0, 90.0, 2);
        EXPECT_EQ(ids.size(), 8u); // TODO: will be 2 when turn on spatialite
    }
    
    {
        PointTable table;
        db.setupPointTableFromTileSet(tileSetIds[0], table);
        
        Stage* stage = db.query(table, tileSetIds[0], 0.0, 0.0, 180.0, 90.0, 2);
        
        // execution
        stage->prepare(table);
        PointViewSet outputViews = stage->execute(table);
        
        EXPECT_EQ(outputViews.size(), 1u);
        PointViewPtr outView = *(outputViews.begin());
        EXPECT_EQ(outView->size(), 2u);
        testPoint(outView, 0, data[4]);
        testPoint(outView, 1, data[5]);
    }

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
        const std::vector<uint32_t> tileSetIds = db.getTileSetIds();
        const uint32_t tileSetId = tileSetIds[0];
        
        // how many levels do we have?
        rialtosupport::RialtoDb::TileSetInfo tileSetInfo = db.getTileSetInfo(tileSetId);
        const uint32_t bestLevel = tileSetInfo.maxLevel;

        // NOTE: this table/query/stage/execute mechanism will likely be
        // replaced by a formal Reader down the road, in proper pdal fashion

        // get ready to execute...
        PointTable table;
        db.setupPointTableFromTileSet(tileSetId, table);
        PointViewSet views;
        PointViewPtr view;
                
        // do the big expensive query, which builds your pipeline
        // NOTE: api will change, this should be a StagePtr or some such
        // NOTE: api may change, I should use the BBOX class probably
        Stage* stage1 = db.query(table, tileSetId, 0.0, 0.0, 180.0, 90.0, bestLevel);
        
        // NOTE: we'll always return a valid stage, even if we know there are no points
        EXPECT_TRUE(stage1 != NULL);
        
        // go!
        stage1->prepare(table);
        views = stage1->execute(table);
        
        // check our output: we should have one point view, with 2 points in it
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 2u);
        testPoint(view, 0, data[4]);
        testPoint(view, 1, data[5]);
        
        // That was so much fun, let's do it again!
        Stage* stage2 = db.query(table, tileSetId, -180.0, -90.0, 0.0, 0.0, bestLevel);
        stage2->prepare(table);
        views = stage2->execute(table);

        // check output again
        EXPECT_EQ(views.size(), 1u);
        view = *(views.begin());
        EXPECT_EQ(view->size(), 2u);
        testPoint(view, 0, data[2]);
        testPoint(view, 1, data[3]);
    }

    //FileUtils::deleteDirectory(Support::temppath("oscar.sqlite"));
}
