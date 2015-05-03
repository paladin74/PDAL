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

    // TODO: for now, we only support two tiles at the root
    m_roots = new tilercommon::Tile*[2];
    const tilercommon::Rectangle r00(-180.0, -90.0, 0.0, 90.0); // wsen
    const tilercommon::Rectangle r10(0.0, -90.0, 180.0, 90.0);
    m_roots[0] = new tilercommon::Tile(*this, 0, 0, 0, r00);
    m_roots[1] = new tilercommon::Tile(*this, 0, 1, 0, r10);
}


TileSet::~TileSet()
{
    if (m_roots) {
        delete m_roots[0];
        delete m_roots[1];
        delete[] m_roots;
    }
}


void TileSet::ready(PointTableRef table)
{
    printf("{TileSet::ready}\n");

    MetadataNode root = table.metadata();

    assert(root.valid());
    MetadataNode tileSetNode = root.addList("tileSet");
    assert(tileSetNode.valid());

    // TODO: hard-coded for now
    tileSetNode.add("maxLevel", m_maxLevel);
    tileSetNode.add("numCols", 2);
    tileSetNode.add("numRows", 1);
    tileSetNode.add("minX", -180.0);
    tileSetNode.add("minY", -90.0);
    tileSetNode.add("maxX", 180.0);
    tileSetNode.add("maxY", 90.0);

    m_metadata = tileSetNode;

    setStatsMetadata(table.metadata());
}


void TileSet::run(PointViewPtr sourceView, PointViewSet* outputSet)
{
    printf("{TileSet::run}\n");

    m_sourceView = sourceView;
    m_outputSet = outputSet;

    // enter each point into the tile set
    for (PointId idx = 0; idx < sourceView->size(); ++idx)
    {
        const double lon = sourceView->getFieldAs<double>(Dimension::Id::X, idx);
        const double lat = sourceView->getFieldAs<double>(Dimension::Id::Y, idx);
        addPoint(idx, lon, lat);
    }

    //std::vector<uint32_t> numTilesPerLevel(m_maxLevel+1);
    //std::vector<uint64_t> numPointsPerLevel(m_maxLevel+1);
    //m_roots[0]->collectCounts(numTilesPerLevel, numPointsPerLevel);
    //m_roots[1]->collectCounts(numTilesPerLevel, numPointsPerLevel);

    setTileSetMetadata();
}

void TileSet::done(PointTableRef table)
{
    printf("{TileSet::done}\n");
    return;
}


void TileSet::setStatsMetadata(const MetadataNode& root)
{
    assert(m_metadata.valid());
    MetadataNode records = m_metadata.addList("stats");

    MetadataNode statsNode = root.findChild("filters.stats");
    assert(statsNode.valid());

    for (auto node1 : statsNode.children("statistic"))
    {
        assert(node1.valid());
        //printf("=>%s %s\n", node1.name().c_str(), node1.value().c_str());

        for (auto node2 : node1.children())
        {
            assert(node2.valid());
            //printf("==>>%s %s\n", node2.name().c_str(), node2.value().c_str());

            if (node2.name().compare("name")==0) {
                std::string name = node2.value();
                MetadataNode avg = node1.findChild("average");
                MetadataNode min = node1.findChild("minimum");
                MetadataNode max = node1.findChild("maximum");
                if (!avg.valid() || !min.valid() || !max.valid()) {
                    throw pdal_error("TileFilter: required stats metadata not found");
                }
                std::string avgS = avg.value();
                std::string minS = min.value();
                std::string maxS = max.value();
                //printf("%s: %s / %s / %s\n", name.c_str(), minS.c_str(), avgS.c_str(), maxS.c_str());

                MetadataNode thisRecord = records.addList(name);
                thisRecord.add("min", minS);
                thisRecord.add("avg", avgS);
                thisRecord.add("max", maxS);
            }
        }
    }
}


PointViewPtr TileSet::createPointView()
{
    PointViewPtr p = m_sourceView->makeNew();
    m_outputSet->insert(p);

    return p;
}


// enter the point into the tree
void TileSet::addPoint(PointId idx, double lon, double lat)
{
    if (lon < 0)
        m_roots[0]->add(m_sourceView, idx, lon, lat);
    else
        m_roots[1]->add(m_sourceView, idx, lon, lat);
}


// set the metadata for each tree node
void TileSet::setTileSetMetadata()
{
    assert(m_metadata.valid());

    // TODO: hard-coded for now
    m_metadata.add("numCols", 2);
    m_metadata.add("numRows", 1);
    m_metadata.add("minX", -180.0);
    m_metadata.add("minY", -90.0);
    m_metadata.add("maxX", 180.0);
    m_metadata.add("maxY", 90.0);

    MetadataNode tilesNode = m_metadata.addList("tiles");
    assert(tilesNode.valid());

    m_roots[0]->setTileMetadata(tilesNode);
    m_roots[1]->setTileMetadata(tilesNode);
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


// set the metaadata for this node, and then recurse down
void Tile::setTileMetadata(MetadataNode& tileSetNode)
{
  // child mask
  uint32_t mask = 0x0;
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
      if (m_children[0]) m_children[0]->setTileMetadata(tileSetNode);
      if (m_children[1]) m_children[1]->setTileMetadata(tileSetNode);
      if (m_children[2]) m_children[2]->setTileMetadata(tileSetNode);
      if (m_children[3]) m_children[3]->setTileMetadata(tileSetNode);
    }
}


// Add the point to this tile.
//
// If we're not a leaf tile, add the point only if we're a module-N numbered
// point, where N is based on the level.
//
// If we're not a leaf tile, add the node to one of our child tiles.
void Tile::add(PointViewPtr sourcePointView, PointId pointNumber, double lon, double lat)
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


void Tile::collectCounts(std::vector<uint32_t>& numTilesPerLevel, std::vector<uint64_t>& numPointsPerLevel) const
{
    if (m_pointView) {
        numPointsPerLevel[m_level] += m_pointView->size();
    }
    numTilesPerLevel[m_level]++;

    if (m_children)
    {
        for (int i=0; i<4; ++i)
        {
            if (m_children[i])
            {
                m_children[i]->collectCounts(numTilesPerLevel, numPointsPerLevel);
            }
        }
    }
}


} // namespace tilercommon

} // namespace pdal
