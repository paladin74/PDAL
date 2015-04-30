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
    m_maxLevel = options.getValueOrDefault<uint32_t>("maxLevel");
    
    // TODO: make these be options
    m_numTilesX = 2;    
    m_numTilesY = 1;

    // TODO: make this be an option
    m_rectangle = tilercommon::Rectangle(-180.0, -90.0, 180.0, 90.0);

    m_roots = new tilercommon::Tile*[2];

    const tilercommon::Rectangle r00(-180.0, -90.0, 0.0, 90.0); // wsen
    const tilercommon::Rectangle r10(0.0, -90.0, 180.0, 90.0);
    m_roots[0] = new tilercommon::Tile(0, 0, 0, r00, m_maxLevel, log());
    m_roots[1] = new tilercommon::Tile(0, 1, 0, r10, m_maxLevel, log());
}


Options TilerFilter::getDefaultOptions()
{
    Options options;
    
    options.add("maxLevel", 23, "Number of levels"); // yields 0..23

    return options;
}


bool TilerFilter::isMetadataValid(const PointViewSet& viewSet)
{
    if (viewSet.size() == 0) return true;
    
    PointViewPtr view = *(viewSet.begin());
    const MetadataNode node = view->metadata().findChild("tiles");
    if (!node.valid()) return false;
    
    const MetadataNodeList children = node.children();    
    if (children.size() != viewSet.size()) return false;

    return true;
}


PointViewSet TilerFilter::run(PointViewPtr inView)
{
    // TODO: assert the input is ESPG:4326

    PointViewSet viewSet;

    const PointView& inViewRef(*inView.get());

    // build the tiles
    for (PointId idx = 0; idx < inViewRef.size(); ++idx)
    {
        double lon = inViewRef.getFieldAs<double>(Dimension::Id::X, idx);
        double lat = inViewRef.getFieldAs<double>(Dimension::Id::Y, idx);

        if (lon < 0)
            m_roots[0]->add(inViewRef, idx, lon, lat, viewSet);
        else
            m_roots[1]->add(inViewRef, idx, lon, lat, viewSet);
    }

    assert(isMetadataValid(viewSet));

    return viewSet;
}

} // pdal
