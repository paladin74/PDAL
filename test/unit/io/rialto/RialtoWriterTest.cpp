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
    writerOptions.add("filename", Support::temppath("RialtoTest"));
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

    /*bool ok = Support::compare_text_files(Support::temppath("RialtoTest/header.json"),
                                          Support::datapath("io/header.json"));
    if (ok)
        FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));
    EXPECT_TRUE(ok);*/
}


/*
TEST(RialtoWriterTest, testWriteHeaderOverwrite)
{
    FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

    BOX3D bounds(1.0, 2.0, 3.0, 11.0, 12.0, 13.0);

    Options ro;
    ro.add("bounds", bounds);
    ro.add("count", 10);
    ro.add("mode", "ramp");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    reader->setOptions(ro);

    Options wo;
    wo.add("filename", Support::temppath("RialtoTest"));
    wo.add("max_level", 0);
    wo.add("overwrite", true);

    std::unique_ptr<Stage> writer(f.createStage("writers.rialto"));
    writer->setOptions(wo);
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);
    writer->execute(table);

    bool ok = Support::compare_text_files(Support::temppath("RialtoTest/header.json"),
                                          Support::datapath("io/header.json"));

    if (ok)
        FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

    EXPECT_TRUE(ok);
}

TEST(RialtoWriterTest, testWriteHeaderNoOverwrite)
{
    FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

    BOX3D bounds(1.0, 2.0, 3.0, 11.0, 12.0, 13.0);

    Options ro;
    ro.add("bounds", bounds);
    ro.add("count", 10);
    ro.add("mode", "ramp");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    reader->setOptions(ro);

    Options wo;
    wo.add("filename", Support::temppath("RialtoTest"));
    wo.add("max_level", 0);
    wo.add("overwrite", false);

    std::unique_ptr<Stage> writer(f.createStage("writers.rialto"));
    writer->setOptions(wo);
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);
    EXPECT_THROW(writer->execute(table), pdal_error);
}
*/
