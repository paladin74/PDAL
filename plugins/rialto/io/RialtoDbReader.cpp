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


RialtoDbReader::RialtoDbReader() :
    Reader()
{}


void RialtoDbReader::initialize()
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::initialize()" << std::endl;
    
    if (!m_db.get())
    {
        m_db = std::unique_ptr<RialtoDb>(new RialtoDb(m_filename, log()));
        m_db->open(false);
    
        std::vector<uint32_t> ids;
        m_db->readTileSetIds(ids);
        assert(ids.size()==1); // TODO: always take the first one for now
        m_tileSetId = ids[0];
    
        m_tileSetInfo = std::unique_ptr<RialtoDb::TileSetInfo>(new RialtoDb::TileSetInfo());

        m_db->readTileSetInfo(m_tileSetId, *m_tileSetInfo);
    
        m_level = m_tileSetInfo->maxLevel;
    }
}


Options RialtoDbReader::getDefaultOptions()
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::getDefaultOptions()" << std::endl;

    Options options;

    return options;
}


void RialtoDbReader::processOptions(const Options& options)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::processOptions()" << std::endl;
    
    static const double minx = -179.9;
    static const double miny = -89.9;
    static const double maxx = 179.9;
    static const double maxy = 89.9;
    static const double minz = (std::numeric_limits<double>::lowest)();
    static const double maxz = (std::numeric_limits<double>::max)();
    static const BOX3D all(minx, miny, minz, maxx, maxy, maxz);
    
    m_query = options.getValueOrDefault<BOX3D>("bbox", all);
    
    log()->get(LogLevel::Debug) << "process options: bbox="
        << m_query << std::endl;
}


void RialtoDbReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::addDimensions()" << std::endl;

    m_db->setupLayout(*m_tileSetInfo, layout);
}


void RialtoDbReader::ready(PointTableRef table)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::ready()" << std::endl;
    // TODO: anything to do here?
}


point_count_t RialtoDbReader::read(PointViewPtr view, point_count_t count)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::read()" << std::endl;

    // TODO: `count` is ignored
    
    const double minx = m_query.minx;
    const double miny = m_query.miny;
    const double maxx = m_query.maxx;
    const double maxy = m_query.maxy;

#if 0
    std::vector<uint32_t> ids;
    m_db->queryForTileIds(m_tileSetId, minx, miny, maxx, maxy, m_level, ids);

    for (auto id: ids) 
    {
        RialtoDb::TileInfo info;
        m_db->readTileInfo(id, true, info);
        
        log()->get(LogLevel::Debug) << "  got some points: " << info.numPoints << std::endl;

        m_db->serializeToPointView(info, view);

        log()->get(LogLevel::Debug) << "  view now has this many: " << view->size() << std::endl;
    }
#else
    std::vector<uint32_t> ids;
    m_db->queryForTileInfosBegin(m_tileSetId, minx, miny, maxx, maxy, m_level);

    RialtoDb::TileInfo info;
    do {    
        bool ok = m_db->queryForTileInfos(info);
        if (!ok) break;
        
        log()->get(LogLevel::Debug) << "  got some points: " << info.numPoints << std::endl;

        m_db->serializeToPointView(info, view);

        log()->get(LogLevel::Debug) << "  view now has this many: " << view->size() << std::endl;
    } while (m_db->queryForTileInfosNext());
#endif
  
    return view->size();
}

} // namespace pdal
