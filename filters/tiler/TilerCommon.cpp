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

#include "TilerCommon.hpp"

#include <pdal/Log.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>

#include <cmath>

namespace pdal
{
namespace tilercommon
{
    
Tile::Tile(
        uint32_t level,
        uint32_t tx,
        uint32_t ty,
        Rectangle r,
        uint32_t maxLevel,
        PointViewSet& pointViewSet,
        LogPtr log) :
    m_level(level),
    m_tileX(tx),
    m_tileY(ty),
    m_children(NULL),
    m_rect(r),
    m_maxLevel(maxLevel),
    m_skip(0),
    m_pointViewSet(pointViewSet),
    m_pointView(NULL),
    m_log(log)
{
    assert(m_maxLevel <= 32);
    assert(m_level <= m_maxLevel);

    m_log->get(LogLevel::Debug1) << "created tb (l=" << m_level
        << ", tx=" << m_tileX
        << ", ty=" << m_tileY
        << ") (slip" << m_skip
        << ")  --  w" << m_rect.west()
        << " s" << m_rect.south()
        << " e" << m_rect.east()
        << " n" << m_rect.north() << "\n";

    // level N+1 has 1/4 the points of level N
    //
    // level 3: skip 1
    // level 2: skip 4
    // level 1: skip 16
    // level 0: skip 256

    // max=3, max-level=u
    // 3-3=0  skip 1   4^0
    // 3-2=1  skip 4    4^1
    // 3-1=2  skip 16    4^2
    // 3-0=3  skip 64    4^3
    //
    m_skip = std::pow(4, (m_maxLevel - m_level));
    m_log->get(LogLevel::Debug1) << "level=" << m_level
        << "  skip=" << m_skip << "\n";
}


Tile::~Tile()
{
    if (m_children != NULL)
    {
        for (int i=0; i<4; ++i)
        {
            if (m_children[i])
            {
                delete m_children[i];
            }
        }
        delete[] m_children;
    }
}


void Tile::createPointView(const PointView& sourcePointView)
{
    m_pointView = sourcePointView.makeNew();

    MetadataNode node = m_pointView->metadata().findChild("tiles");
    if (!node.valid()) {
        node = m_pointView->metadata().add("tiles");
    }

    const std::string idString = std::to_string(m_pointView->id());

    MetadataNode tilesNode = node.add(idString);
    tilesNode.add("level", m_level);
    tilesNode.add("tileX", m_tileX);
    tilesNode.add("tileY", m_tileY);

    //printf("made view for %u %u %u\n", m_level, m_tileX, m_tileY);
}


void Tile::add(const PointView& sourcePointView, PointId pointNumber, double lon, double lat)
{
    assert(m_rect.contains(lon, lat));

    m_log->get(LogLevel::Debug5) << "-- -- " << pointNumber
        << " " << m_skip
        << " " << (pointNumber % m_skip == 0) << "\n";

    // put the point into this tile, if we're at the right level of decimation
    if (pointNumber % m_skip == 0)
    {
        if (!m_pointView) {
            createPointView(sourcePointView);
            m_pointViewSet.insert(m_pointView);
        }
        
        //printf("%u:  to %u,%u,%u   ===   %u   ===   %lf %lf\n", m_pointView->id(), m_level, m_tileX, m_tileY, pointNumber, lon, lat);
        m_pointView->appendPoint(sourcePointView, pointNumber);
    }

    if (m_level == m_maxLevel) return;

    if (!m_children)
    {
        m_children = new Tile*[4];
        m_children[0] = NULL;
        m_children[1] = NULL;
        m_children[2] = NULL;
        m_children[3] = NULL;
    }

    Rectangle::Quadrant q = m_rect.getQuadrantOf(lon, lat);
    m_log->get(LogLevel::Debug5) << "which=" << q << "\n";

    Tile* child = m_children[q];
    if (child == NULL)
    {
        Rectangle r = m_rect.getQuadrantRect(q);
        switch (q)
        {
            case Rectangle::QuadrantSW:
                child = new Tile(m_level+1, m_tileX*2, m_tileY*2+1, r, m_maxLevel, m_pointViewSet, m_log);
                break;
            case Rectangle::QuadrantNW:
                child = new Tile(m_level+1, m_tileX*2, m_tileY*2, r, m_maxLevel, m_pointViewSet, m_log);
                break;
            case Rectangle::QuadrantSE:
                child = new Tile(m_level+1, m_tileX*2+1, m_tileY*2+1, r, m_maxLevel, m_pointViewSet, m_log);
                break;
            case Rectangle::QuadrantNE:
                child = new Tile(m_level+1, m_tileX*2+1, m_tileY*2, r, m_maxLevel, m_pointViewSet, m_log);
                break;
            default:
                throw pdal_error("invalid quadrant");
        }
        m_children[q] = child;
    }

    child->add(sourcePointView, pointNumber, lon, lat);
}


void Tile::collectStats(std::vector<uint32_t> numTilesPerLevel, std::vector<uint64_t> numPointsPerLevel) const
{
    numPointsPerLevel[m_level] += m_pointView->size();
    ++numTilesPerLevel[m_level];

    for (int i=0; i<4; ++i)
    {
        if (m_children && m_children[i])
        {
            m_children[i]->collectStats(numTilesPerLevel, numPointsPerLevel);
        }
    }
}


/*void Tile::write(const char* prefix) const
{
    char* filename = new char[strlen(prefix) + 1024];

    sprintf(filename, "%s", prefix);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d", prefix, m_level);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d/%d", prefix, m_level, m_tileX);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d/%d/%d.ria", prefix, m_level, m_tileX, m_tileY);

    m_log->get(LogLevel::Debug1) << "--> " << filename << "\n";

    FILE* fp = fopen(filename, "wb");

    writeData(fp);

    // child mask
    uint8_t mask = 0x0;
    if (m_children)
    {
        if (m_children[QuadSW]) mask += 1;
        if (m_children[QuadSE]) mask += 2;
        if (m_children[QuadNE]) mask += 4;
        if (m_children[QuadNW]) mask += 8;
    }
    fwrite(&mask, 1, 1, fp);

    fclose(fp);

    if (m_children)
    {
        for (int i=0; i<4; ++i)
        {
            if (m_children[i])
            {
                m_children[i]->write(prefix);
            }
        }
    }

    delete[] filename;
}*/


/*void Tile::writeData(FILE* fp) const
{
    const PointLayoutPtr layout(m_table.layout());
    for (size_t i=0; i<m_points.size(); ++i)
    {
        char* p = m_points[i];

        for (const auto& dim : layout->dims())
        {
            size_t size = Dimension::size(layout->dimType(dim));

            fwrite(p, size, 1, fp);

            p += size;
        }
    }
}*/

} // namespace tilercommon

} // namespace pdal
