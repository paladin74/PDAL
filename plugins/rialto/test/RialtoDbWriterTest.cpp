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

using namespace pdal;

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


static void readBytes(const std::string& filename, uint8_t* buf, uint32_t len)
{
    const char* name = Support::temppath(filename).c_str();
    FILE* fp = fopen(name, "rb");
    EXPECT_TRUE(fp != NULL);
    size_t cnt = fread(buf, 1, len, fp);
    EXPECT_EQ(cnt, len);
    fclose(fp);
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
    //FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

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

    Options writerOptions;
    writerOptions.add("filename", Support::temppath("rialto1"));
    writerOptions.add("overwrite", true);

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

    //RialtoFileWriter writer;
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialtodb"));
    writer->setOptions(writerOptions);
    writer->setInput(tiler);


    // execution
    writer->prepare(table);
    PointViewSet outputViews = writer->execute(table);

    bool ok = Support::compare_text_files(Support::temppath("rialto1/header.json-db"),
                                          Support::datapath("io/rialto1-header.json"));
    EXPECT_TRUE(ok);

    verify("rialto1/0/0/0.ria-db", &data[0], 15);
    verify("rialto1/0/1/0.ria-db", NULL, 15);

    verify("rialto1/1/0/0.ria-db", &data[0], 8);
    verify("rialto1/1/0/1.ria-db", NULL, 1);
    verify("rialto1/1/1/0.ria-db", NULL, 4);
    verify("rialto1/1/1/1.ria-db", NULL, 2);
    verify("rialto1/1/2/0.ria-db", &data[4], 2);
    verify("rialto1/1/2/1.ria-db", NULL, 4);
    verify("rialto1/1/3/0.ria-db", NULL, 1);
    verify("rialto1/1/3/1.ria-db", NULL, 8);

    verify("rialto1/2/0/0.ria-db", &data[0], 0);
    verify("rialto1/2/0/3.ria-db", &data[2], 0);
    verify("rialto1/2/3/0.ria-db", &data[1], 0);
    verify("rialto1/2/3/3.ria-db", &data[3], 0);
    verify("rialto1/2/5/1.ria-db", &data[4], 0);
    verify("rialto1/2/5/2.ria-db", &data[6], 0);
    verify("rialto1/2/6/1.ria-db", &data[5], 0);
    verify("rialto1/2/6/2.ria-db", &data[7], 0);

    verifyDirectorySize("rialto1", 4);
    verifyDirectorySize("rialto1/0", 2);
    verifyDirectorySize("rialto1/0/0", 1);
    verifyDirectorySize("rialto1/0/1", 1);
    verifyDirectorySize("rialto1/1", 4);
    verifyDirectorySize("rialto1/1/0", 2);
    verifyDirectorySize("rialto1/1/1", 2);
    verifyDirectorySize("rialto1/1/2", 2);
    verifyDirectorySize("rialto1/1/3", 2);
    verifyDirectorySize("rialto1/2", 4);
    verifyDirectorySize("rialto1/2/0", 2);
    verifyDirectorySize("rialto1/2/3", 2);
    verifyDirectorySize("rialto1/2/5", 2);
    verifyDirectorySize("rialto1/2/6", 2);

    if (ok) {
        //FileUtils::deleteDirectory(Support::temppath("rialto1"));
    }
}
