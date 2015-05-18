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

#include <pdal/pdal.hpp>
#include <../filters/tiler/TilerFilter.hpp>
#include <../filters/stats/StatsFilter.hpp>
#include <../filters/reprojection/ReprojectionFilter.hpp>

#include "Tool.hpp"

// TODO: document usage
// TODO: add stats output
// TODO: add -info option


int main(int argc, char* argv[])
{
    Tool tool;
    
    tool.processOptions(argc, argv);
    
    pdal::Reader* reader = tool.createReader();
    pdal::Writer* writer = tool.createWriter();    
    
    Options reprojOptions;
    ReprojectionFilter reproj;
    reprojOptions.add("out_srs", "EPSG:4326");
    reproj.setOptions(reprojOptions);
    reproj.setInput(*reader);

    Options statsOptions;
    StatsFilter stats;
    stats.setOptions(statsOptions);
    stats.setInput(reproj);

    Options tilerOptions;
    tilerOptions.add("maxLevel", 18);  // TODO
    TilerFilter tiler;
    tiler.setOptions(tilerOptions);
    tiler.setInput(stats);

    writer->setInput(tiler);
    
    pdal::PointTable table;
    writer->prepare(table);
    writer->execute(table);

    tool.verify();
    
    delete writer;
    delete reader;
    
    return 0;
}
