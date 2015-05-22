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

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.rialtodb",
    "Read data from a Rialto DB",
    "" );

CREATE_SHARED_PLUGIN(1, 0, rialto::RialtoDbReader, Reader, s_info)

namespace rialto
{


std::string RialtoDbReader::getName() const { return s_info.name; }


RialtoDbReader::RialtoDbReader() :
    Reader(),
    m_db(NULL)
{}


RialtoDbReader::~RialtoDbReader()
{
    if (m_db)
    {
        m_db->close();
        delete m_db;
    }
}


void RialtoDbReader::initialize()
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::initialize()" << std::endl;

    if (!m_db)
    {
        m_db = new RialtoDb(m_filename, log());
        m_db->open(false);

        std::vector<std::string> names;
        m_db->readTileTableNames(names);
        m_tileTableName = names[0];
        assert(names.size()==1); // TODO: always take the first one for now

        m_tileTableInfo = std::unique_ptr<TileTableInfo>(new TileTableInfo());

        m_db->readTileTable(m_tileTableName, *m_tileTableInfo);
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
    m_filename = options.getValueOrThrow<std::string>("filename");

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

    m_level = options.getValueOrDefault<uint32_t>("level", 9999);
}


void RialtoDbReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::addDimensions()" << std::endl;

    m_db->setupLayout(*m_tileTableInfo, layout);
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

    uint32_t maxLevel = m_level;
    if (maxLevel == 9999) // TODO
    {
        maxLevel = m_tileTableInfo->getMaxLevel();
    }

    m_db->queryForTiles_begin(m_tileTableName, minx, miny, maxx, maxy, maxLevel);

    TileInfo info;

    do {
        bool ok = m_db->queryForTiles_step(info);
        if (!ok) break;

        log()->get(LogLevel::Debug) << "  got some points: " << info.getNumPoints() << std::endl;


        PointViewPtr tempView = view->makeNew();

        info.getPatch().exportToPV(info.getNumPoints(), tempView);

        for (uint32_t i=0; i<tempView->size(); i++) {
            const double x = tempView->getFieldAs<double>(Dimension::Id::X, i);
            const double y = tempView->getFieldAs<double>(Dimension::Id::Y, i);
            if (x >= minx && x <= maxx && y >= miny && y <= maxy)
            {
                view->appendPoint(*tempView, i);
            }
        }

        log()->get(LogLevel::Debug) << "  view now has this many: " << view->size() << std::endl;
    } while (m_db->queryForTiles_next());
  
    return view->size();
}

} // namespace rialto

void Options::remove(const std::string& name)
{
    m_options.erase(name);
}
} // namespace pdal
