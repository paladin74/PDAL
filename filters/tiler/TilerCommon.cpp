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


TileSet::TileSet(
        uint32_t maxLevel,
        LogPtr log) :
    m_sourceView(NULL),
    m_outputSet(NULL),
    m_maxLevel(maxLevel),
    m_log(log),
    m_roots(NULL)

{
    assert(m_maxLevel <= 32);
}


TileSet::~TileSet()
{
    if (m_roots) {
        delete m_roots[0];
        delete m_roots[1];
        delete[] m_roots;
    }
}


void TileSet::prep(const PointView* sourceView, PointViewSet* outputSet)
{
    m_sourceView = sourceView;
    m_outputSet = outputSet;

    m_roots = new tilercommon::Tile*[2];

    const tilercommon::Rectangle r00(-180.0, -90.0, 0.0, 90.0); // wsen
    const tilercommon::Rectangle r10(0.0, -90.0, 180.0, 90.0);
    m_roots[0] = new tilercommon::Tile(*this, 0, 0, 0, r00);
    m_roots[1] = new tilercommon::Tile(*this, 0, 1, 0, r10);
}


void TileSet::addPoint(PointId idx, double lon, double lat)
{
    if (lon < 0)
        m_roots[0]->add(m_sourceView, idx, lon, lat);
    else
        m_roots[1]->add(m_sourceView, idx, lon, lat);
}


void TileSet::setMetadata(MetadataNode& root)
{
    assert(root.valid());
    MetadataNode tileSetNode = root.addList("tileSet");
    assert(tileSetNode.valid());

    m_roots[0]->setMetadata(tileSetNode);
    m_roots[1]->setMetadata(tileSetNode);
}


PointViewPtr TileSet::createPointView()
{
    PointViewPtr p = m_sourceView->makeNew();
    m_outputSet->insert(p);

    return p;
}


Tile::Tile(
        TileSet& tileSet,
        uint32_t level,
        uint32_t tx,
        uint32_t ty,
        Rectangle r) :
    m_tileSet(tileSet),
    m_level(level),
    m_tileX(tx),
    m_tileY(ty),
    m_children(NULL),
    m_rect(r),
    m_skip(0),
    m_pointView(NULL)
{
    static int lastId = 0;
    m_id = lastId++;

    assert(m_level <= m_tileSet.getMaxLevel());

    log()->get(LogLevel::Debug1) << "created tb (l=" << m_level
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
    m_skip = std::pow(4, (m_tileSet.getMaxLevel() - m_level));
    log()->get(LogLevel::Debug1) << "level=" << m_level
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


void Tile::setMetadata(MetadataNode& tileSetNode)
{
  // child mask
  uint8_t mask = 0x0;
  if (m_children)
  {
      if (m_children[Rectangle::QuadrantSW]) mask += 1;
      if (m_children[Rectangle::QuadrantSE]) mask += 2;
      if (m_children[Rectangle::QuadrantNE]) mask += 4;
      if (m_children[Rectangle::QuadrantNW]) mask += 8;
  }

  const std::string idString = std::to_string(m_id);
  MetadataNode tileNode = tileSetNode.addList(idString);

  tileNode.add("level", m_level);
  tileNode.add("tileX", m_tileX);
  tileNode.add("tileY", m_tileY);
  tileNode.add("mask", mask);

  if (m_pointView)
  {
    tileNode.add("pointView", m_pointView->id());
  }

  if (m_children) {
      if (m_children[0]) m_children[0]->setMetadata(tileSetNode);
      if (m_children[1]) m_children[1]->setMetadata(tileSetNode);
      if (m_children[2]) m_children[2]->setMetadata(tileSetNode);
      if (m_children[3]) m_children[3]->setMetadata(tileSetNode);
    }
}


void Tile::add(const PointView* sourcePointView, PointId pointNumber, double lon, double lat)
{
    assert(m_rect.contains(lon, lat));

    log()->get(LogLevel::Debug5) << "-- -- " << pointNumber
        << " " << m_skip
        << " " << (pointNumber % m_skip == 0) << "\n";

    // put the point into this tile, if we're at the right level of decimation
    if (pointNumber % m_skip == 0)
    {
        if (!m_pointView) {
            m_pointView = m_tileSet.createPointView();
        }
        assert(m_pointView);

        m_pointView->appendPoint(*sourcePointView, pointNumber);
    }

    if (m_level == m_tileSet.getMaxLevel()) return;

    if (!m_children)
    {
        m_children = new Tile*[4];
        m_children[0] = NULL;
        m_children[1] = NULL;
        m_children[2] = NULL;
        m_children[3] = NULL;
    }

    Rectangle::Quadrant q = m_rect.getQuadrantOf(lon, lat);
    log()->get(LogLevel::Debug5) << "which=" << q << "\n";

    Tile* child = m_children[q];
    if (child == NULL)
    {
        Rectangle r = m_rect.getQuadrantRect(q);
        switch (q)
        {
            case Rectangle::QuadrantSW:
                child = new Tile(m_tileSet, m_level+1, m_tileX*2, m_tileY*2+1, r);
                break;
            case Rectangle::QuadrantNW:
                child = new Tile(m_tileSet, m_level+1, m_tileX*2, m_tileY*2, r);
                break;
            case Rectangle::QuadrantSE:
                child = new Tile(m_tileSet, m_level+1, m_tileX*2+1, m_tileY*2+1, r);
                break;
            case Rectangle::QuadrantNE:
                child = new Tile(m_tileSet, m_level+1, m_tileX*2+1, m_tileY*2, r);
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


// TODO: not const due to use of log()
void Tile::write(const char* prefix)
{
    char* filename = new char[strlen(prefix) + 1024];

    sprintf(filename, "%s", prefix);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d", prefix, m_level);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d/%d", prefix, m_level, m_tileX);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d/%d/%d.ria", prefix, m_level, m_tileX, m_tileY);

    log()->get(LogLevel::Debug1) << "--> " << filename << "\n";

    FILE* fp = fopen(filename, "wb");

    writeData(fp);

    // child mask
    uint8_t mask = 0x0;
    if (m_children)
    {
        if (m_children[Rectangle::QuadrantSW]) mask += 1;
        if (m_children[Rectangle::QuadrantSE]) mask += 2;
        if (m_children[Rectangle::QuadrantNE]) mask += 4;
        if (m_children[Rectangle::QuadrantNW]) mask += 8;
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
}


char* Tile::getPointData(const PointView& buf, PointId& idx) const
{
    char* p = new char[buf.pointSize()];
    char* q = p;

    for (const auto& dim : buf.dims())
    {
        buf.getRawField(dim, idx, q);
        q += buf.dimSize(dim);
    }

    return p;
}


void Tile::writeData(FILE* fp) const
{
    //const PointLayoutPtr layout(m_pointView.layout());

    for (size_t i=0; i<m_pointView->size(); ++i)
    {
        PointId idx = i;
        char* p = getPointData(*m_pointView, idx);

        for (const auto& dim : m_pointView->dimTypes())
        {
            size_t size = Dimension::size(dim.m_type);

            fwrite(p, size, 1, fp);

            p += size;
        }
    }
}

} // namespace tilercommon

} // namespace pdal
