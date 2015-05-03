#include <pdal/pdal_test_main.hpp>

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include <pdal/BufferReader.hpp>
#include <TilerFilter.hpp>
#include <StatsFilter.hpp>

#include "RialtoWriter.hpp"
#include "Support.hpp"

using namespace pdal;

TEST(RialtoWriterTest, createWriter)
{
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialto"));
    EXPECT_TRUE(writer.get());
}

TEST(RialtoWriterTest, testConstructor)
{
    std::unique_ptr<RialtoWriter> writer(new RialtoWriter);
    EXPECT_EQ(writer->getName(), "writers.rialto");
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


static void verify(const std::string& filename, uint8_t expectedSize, uint8_t expectedMask, int dataIndex)
{
  uint32_t actualSize = FileUtils::fileSize(Support::temppath(filename));
  EXPECT_EQ(actualSize, expectedSize);

  if (actualSize != 1) return;

  uint8_t actualMask;
  readBytes(filename, &actualMask, 1);
  EXPECT_EQ(actualMask, expectedMask);
}


TEST(RialtoWriterTest, testWriter)
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

    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reader);

    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);

    RialtoWriter writer;
    writer.setOptions(writerOptions);
    writer.setInput(tiler);


    // execution
    writer.prepare(table);
    PointViewSet outputViews = writer.execute(table);

    bool ok = Support::compare_text_files(Support::temppath("rialto1/header.json"),
                                          Support::datapath("io/rialto-header.json"));
    EXPECT_TRUE(ok);

    // TODO: add actual point checks
    verify("rialto1/0/0/0.ria", 25, 15, 0);
    verify("rialto1/0/1/0.ria", 1, 15, -1);

    verify("rialto1/1/0/0.ria", 25, 8, 0);
    verify("rialto1/1/0/1.ria", 1, 1, -1);
    verify("rialto1/1/1/0.ria", 1, 4, -1);
    verify("rialto1/1/1/1.ria", 1, 2, -1);
    verify("rialto1/1/2/0.ria", 25, 3, 4);
    verify("rialto1/1/2/1.ria", 1, 4, -1);
    verify("rialto1/1/3/0.ria", 1, 1, -1);
    verify("rialto1/1/3/1.ria", 1, 8, -1);

    verify("rialto1/2/0/0.ria", 25, 5, 0);
    verify("rialto1/2/0/3.ria", 25, 7, 2);
    verify("rialto1/2/3/0.ria", 25, 6, 1);
    verify("rialto1/2/3/3.ria", 25, 8, 3);
    verify("rialto1/2/5/1.ria", 25, 10, 4);
    verify("rialto1/2/5/2.ria", 25, 12, 6);
    verify("rialto1/2/6/1.ria", 25, 11, 5);
    verify("rialto1/2/6/2.ria", 25, 13, 7);

    if (ok) {
        //FileUtils::deleteDirectory(Support::temppath("rialto1"));
    }
}
