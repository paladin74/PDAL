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
#include "RialtoTest.hpp"

using namespace pdal;
using namespace rialto;

TEST(RialtoFileWriterTest, createWriter)
{
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialtofile"));
    EXPECT_TRUE(writer.get());
}




static void readBytes(const std::string& filename, uint8_t* buf, uint32_t len)
{
    const std::string name = Support::temppath(filename);
    FILE* fp = fopen(name.c_str(), "rb");
    EXPECT_TRUE(fp != NULL);
    size_t cnt = fread(buf, 1, len, fp);
    EXPECT_EQ(cnt, len);
    fclose(fp);
}


static void verifyFile(const std::string& filename,
                   const RialtoTest::Data* expectedData,
                   uint8_t expectedMask)
{
    uint32_t actualSize = FileUtils::fileSize(Support::temppath(filename));

    if (expectedData) {
        std::vector<unsigned char> buf(25);
        readBytes(filename, &buf[0], 25);
        RialtoTest::verifyPointFromBuffer(buf, *expectedData);
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


TEST(RialtoFileWriterTest, testWriter)
{
    const std::string dir(Support::temppath("rialto1"));
    
    FileUtils::deleteDirectory(Support::temppath("rialto1"));

    // set up test data
    // set up test data
    PointTable table;
    PointViewPtr inputView(new PointView(table));
    RialtoTest::Data* actualData = RialtoTest::sampleDataInit(table, inputView);
    
    RialtoTest::createTileFiles(table, inputView, dir);

    bool ok = Support::compare_text_files(Support::temppath("rialto1/header.json"),
                                          Support::datapath("io/rialto1-header.json"));
    EXPECT_TRUE(ok);

    verifyFile("rialto1/0/0/0.ria", &actualData[0], 15);
    verifyFile("rialto1/0/1/0.ria", NULL, 15);

    verifyFile("rialto1/1/0/0.ria", &actualData[0], 8);
    verifyFile("rialto1/1/0/1.ria", NULL, 1);
    verifyFile("rialto1/1/1/0.ria", NULL, 4);
    verifyFile("rialto1/1/1/1.ria", NULL, 2);
    verifyFile("rialto1/1/2/0.ria", &actualData[4], 2);
    verifyFile("rialto1/1/2/1.ria", NULL, 4);
    verifyFile("rialto1/1/3/0.ria", NULL, 1);
    verifyFile("rialto1/1/3/1.ria", NULL, 8);

    verifyFile("rialto1/2/0/0.ria", &actualData[0], 0);
    verifyFile("rialto1/2/0/3.ria", &actualData[2], 0);
    verifyFile("rialto1/2/3/0.ria", &actualData[1], 0);
    verifyFile("rialto1/2/3/3.ria", &actualData[3], 0);
    verifyFile("rialto1/2/5/1.ria", &actualData[4], 0);
    verifyFile("rialto1/2/5/2.ria", &actualData[6], 0);
    verifyFile("rialto1/2/6/1.ria", &actualData[5], 0);
    verifyFile("rialto1/2/6/2.ria", &actualData[7], 0);

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

    delete[] actualData;
    
    FileUtils::deleteDirectory(Support::temppath("rialto1"));
}
