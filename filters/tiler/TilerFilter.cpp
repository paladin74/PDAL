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
}


Options TilerFilter::getDefaultOptions()
{
    Options options;

    options.add("maxLevel", 23, "Number of highest level"); // yields 0..23

    return options;
}


PointViewSet TilerFilter::run(PointViewPtr sourceView)
{
    // TODO: assert the input is ESPG:4326

    PointViewSet outputSet;
    const PointView& sourceViewRef(*sourceView.get());

    tilercommon::TileSet tileSet(sourceViewRef, outputSet, m_maxLevel, log());

    // enter each point into the tile set
    for (PointId idx = 0; idx < sourceViewRef.size(); ++idx)
    {
        const double lon = sourceViewRef.getFieldAs<double>(Dimension::Id::X, idx);
        const double lat = sourceViewRef.getFieldAs<double>(Dimension::Id::Y, idx);
        tileSet.addPoint(idx, lon, lat);
    }

    // Set the metadata for the tile set
    //    tileset:
    //        <tile id>:
    //            level = <l>
    //            tileX = <x>
    //            tileY = <y>
    //            mask = <m>
    //            pointView = <point view id>  # optional
    //        <tile id>:
    //            ...
    //
    // Note the point view node will only be present if the tile contains
    // any points.
    {
      assert(outputSet.size() != 0);
      PointViewPtr tmp = *(outputSet.begin());
      MetadataNode root = tmp->metadata();
      assert(root.valid());

      tileSet.setMetadata(root);
    }

    return outputSet;
}

} // pdal
