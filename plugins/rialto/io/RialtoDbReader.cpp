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

#include "RialtoDbReader.hpp"
#include "RialtoDb.hpp"
#include <pdal/PointView.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.rialtodb",
    "Read data from a Rialto DB",
    "" );

CREATE_SHARED_PLUGIN(1, 0, RialtoDbReader, Reader, s_info)

std::string RialtoDbReader::getName() const { return s_info.name; }

void RialtoDbReader::initialize()
{
    m_db = new RialtoDb(m_filename);
    m_db->open(false);
    m_tileSetId = 1;
}


Options RialtoDbReader::getDefaultOptions()
{
    Options options;

    return options;
}


void RialtoDbReader::processOptions(const Options& options)
{
}


void RialtoDbReader::addDimensions(PointLayoutPtr layout)
{
  PointTable table;
  m_db->setupPointTable(m_tileSetId, table);
  layout->registerDims(table.layout()->dims());
}


void RialtoDbReader::ready(PointTableRef table)
{
}


point_count_t RialtoDbReader::read(PointViewPtr view, point_count_t count)
{
  return 0;
}

} // namespace pdal
