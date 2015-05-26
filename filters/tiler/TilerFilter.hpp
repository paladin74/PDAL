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

#pragma once

#include <pdal/Filter.hpp>

#include "TilerCommon.hpp"

extern "C" int32_t TilerFilter_ExitFunc();
extern "C" PF_ExitFunc TilerFilter_InitPlugin();

namespace pdal
{

// The layout of the metadata for the tile set looks like this:
//
//    tileset:
//        numCols:
//        numRows:
//        minX:
//        minY:
//        maxX:
//        maxY:
//        tiles:
//            <tile id>:
//                level = <l>
//                tileX = <x>
//                tileY = <y>
//                mask = <m>
//                pointView = <point view id>  # optional
//            <tile id>:
//                ...
//
// Note the point view node will only be present if the tile contains
// any points.

class PDAL_DLL TilerFilter : public pdal::Filter
{
public:
    TilerFilter() : Filter(),
        m_tileSet(NULL)
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    void ready(PointTableRef table);
    void done(PointTableRef table);

    Options getDefaultOptions();

private:
    int32_t m_maxLevel; // number of highest level
    int32_t m_numColsAtL0;
    int32_t m_numRowsAtL0;
    double m_minx, m_miny, m_maxx, m_maxy; // bbox of tile tree

    tilercommon::TileSet* m_tileSet;

    virtual void processOptions(const Options& options);
    virtual PointViewSet run(PointViewPtr view);

    TilerFilter& operator=(const TilerFilter&); // not implemented
    TilerFilter(const TilerFilter&); // not implemented
};

} // namespace pdal
