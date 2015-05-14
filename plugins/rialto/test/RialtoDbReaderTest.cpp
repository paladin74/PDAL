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

#include "RialtoTest.hpp"
#include <boost/filesystem.hpp>

#include "../plugins/rialto/io/RialtoDbReader.hpp" // TODO: fix path

using namespace pdal;


TEST(RialtoDbReaderTest, test)
{
    const std::string filename(Support::temppath("rialto4.sqlite"));
    
    FileUtils::deleteFile(filename);

    {
        // set up test database
        PointTable table;
        PointViewPtr inputView(new PointView(table));
        RialtoTest::Data* actualData = RialtoTest::sampleDataInit(table, inputView);
        RialtoTest::createDatabase(table, inputView, filename, 2);
    }

  RialtoDbReader reader;
  Options options;
  options.add("filename", filename);
  //options.add("verbose", LogLevel::Debug);
  reader.setOptions(options);
  
  {
      PointTable table;
      reader.prepare(table);

      PointViewSet viewSet = reader.execute(table);
      EXPECT_EQ(viewSet.size(), 1u);
      PointViewPtr view = *viewSet.begin();
      EXPECT_EQ(view->size(), 8u);
  }

  {
      BOX3D bounds(0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
      options.remove("bbox");
      options.add("bbox", bounds);
      reader.setOptions(options);
  
      PointTable table;
      reader.prepare(table);

      PointViewSet viewSet = reader.execute(table);
      EXPECT_EQ(viewSet.size(), 1u);
      PointViewPtr view = *viewSet.begin();
      EXPECT_EQ(view->size(), 0u);
  }
  
  {
      BOX3D bounds(1.0, 1.0, -10000, 89.0, 89.0, 10000.0);
      options.remove("bbox");
      options.add("bbox", bounds);
      reader.setOptions(options);
  
      PointTable table;
      reader.prepare(table);

      PointViewSet viewSet = reader.execute(table);
      EXPECT_EQ(viewSet.size(), 1u);
      PointViewPtr view = *viewSet.begin();
      EXPECT_EQ(view->size(), 1u);
  }
}
