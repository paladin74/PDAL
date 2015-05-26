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

#include "TilerFilter.hpp"

#include <pdal/pdal_macros.hpp>

#include <iostream>
#include <limits>


// TODO:
//   - need to support wrtiting out only certain levels, e.g. just the
//     highest res, or just levels 10..18, or...
//   - we could make the use of the stats filter be optional -- but right now
//     the rialto client viewer makes good use of this data

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.tiler",
    "Split data up into hiearchical tiles, similar to TMS.",
    "http://pdal.io/stages/filters.tiler.html" );

CREATE_STATIC_PLUGIN(1, 0, TilerFilter, Filter, s_info)

std::string TilerFilter::getName() const { return s_info.name; }

void TilerFilter::processOptions(const Options& options)
{
    m_maxLevel = options.getValueOrThrow<uint32_t>("maxLevel");
    m_numColsAtL0 = options.getValueOrThrow<uint32_t>("numCols");
    m_numRowsAtL0 = options.getValueOrThrow<uint32_t>("numRows");
    m_minx = options.getValueOrThrow<double>("minx");
    m_miny = options.getValueOrThrow<double>("miny");
    m_maxx = options.getValueOrThrow<double>("maxx");
    m_maxy = options.getValueOrThrow<double>("maxy");
    
    if (m_maxLevel >= 64)
    {
        throw pdal_error("TilerFilter: invalid maxLevel must be less than 64");
    }
    if (m_minx >= m_maxx || m_miny >= m_maxy)
    {
        throw pdal_error("TilerFilter: invalid matrix bounding box");
    }
    if (m_numColsAtL0 == 0 || m_numRowsAtL0 == 0)
    {
        throw pdal_error("TilerFilter: invalid matrix dimensions");
    }

}


Options TilerFilter::getDefaultOptions()
{
    Options options;

    options.add("maxLevel", 23, "Number of highest level"); // yields 0..23

    return options;
}


void TilerFilter::ready(PointTableRef table)
{
    assert(m_tileSet == NULL);

    // we require that the stats filter has been run
    const MetadataNode statsNode = table.metadata().findChild("filters.stats");
    if (!statsNode.valid())
    {
        throw pdal_error("TilerFilter: pipeline must have stats filter");
    }

    m_tileSet = new tilercommon::TileSet(m_maxLevel, 
                                         m_minx, m_miny, m_maxx, m_maxy,
                                         m_numColsAtL0, m_numRowsAtL0,
                                         log());

    m_tileSet->ready(table);
}


void TilerFilter::done(PointTableRef table)
{
    m_tileSet->done(table);

    if (m_tileSet)
    {
        delete m_tileSet;
        m_tileSet = NULL;
    }
}


PointViewSet TilerFilter::run(PointViewPtr sourceView)
{
    PointViewSet outputSet;
    const PointView& sourceViewRef(*sourceView.get());

    m_tileSet->run(sourceView, &outputSet);

    return outputSet;
}

} // pdal
