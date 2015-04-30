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
    m_tileSet = NULL;
    
    m_maxLevel = options.getValueOrDefault<uint32_t>("maxLevel");
    
    // TODO: make these be options
    m_numTilesX = 2;    
    m_numTilesY = 1;

    // TODO: make this be an option
    m_rectangle = tilercommon::Rectangle(-180.0, -90.0, 180.0, 90.0);
}


Options TilerFilter::getDefaultOptions()
{
    Options options;
    
    options.add("maxLevel", 23, "Number of levels"); // yields 0..23

    return options;
}


bool TilerFilter::isMetadataValid(const PointViewSet& viewSet)
{
    assert(viewSet.size() != 0);
    
    PointViewPtr view = *(viewSet.begin());
    const MetadataNode node = view->metadata().findChild("tiles");
    assert(node.valid());
    if (!node.valid()) return false;
    
    //const MetadataNodeList children = node.children();    
    //assert(children.size() == viewSet.size());
    //if (children.size() != viewSet.size()) return false;

    return true;
}


void TilerFilter::ready(PointTableRef table)
{
    assert(!m_tileSet);
    m_tileSet = new tilercommon::TileSet(table, m_maxLevel, log());
}


void TilerFilter::done(PointTableRef table)
{
    if (m_tileSet) {
        delete m_tileSet;
        m_tileSet = NULL;
    }
}


PointViewSet TilerFilter::run(PointViewPtr sourceView)
{
    // TODO: assert the input is ESPG:4326

    assert(m_tileSet);
    
    PointViewSet outputSet;

    const PointView& sourceViewRef(*sourceView.get());
        
    m_tileSet->prep(sourceView.get(), &outputSet);

    // build the tiles
    for (PointId idx = 0; idx < sourceViewRef.size(); ++idx)
    {
        double lon = sourceViewRef.getFieldAs<double>(Dimension::Id::X, idx);
        double lat = sourceViewRef.getFieldAs<double>(Dimension::Id::Y, idx);

        m_tileSet->addPoint(idx, lon, lat);
    }

    m_tileSet->setMasks();
    
    assert(isMetadataValid(outputSet));

    delete m_tileSet;
    m_tileSet = NULL;

    return outputSet;
}

} // pdal
