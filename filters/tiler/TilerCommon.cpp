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
    m_roots(NULL),
    m_tileId(0)
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
    m_tableMetadata =  table.metadata();
    assert(m_tableMetadata.valid());
    
    m_tileSetMetadata = m_tableMetadata.findChild("filters.tiler");
    assert(m_tileSetMetadata.valid());
}


void TileSet::run(PointViewPtr sourceView, PointViewSet* outputSet)
{
    m_sourceView = sourceView;
    m_outputSet = outputSet;

    // enter each point into the tile set
    Tile& westTile = *(m_roots[0]);
    Tile& eastTile = *(m_roots[1]);
    
    const uint32_t numPoints = sourceView->size();
    for (PointId idx = 0; idx < numPoints; ++idx)
    {
        const double lon = sourceView->getFieldAs<double>(Dimension::Id::X, idx);
        const double lat = sourceView->getFieldAs<double>(Dimension::Id::Y, idx);
        
        if (lon < 0.0)
            westTile.add(m_sourceView, idx, lon, lat);
        else
            eastTile.add(m_sourceView, idx, lon, lat);
    }

    setHeaderMetadata();
    setStatisticsMetadata();

    {
        westTile.setMask();
        eastTile.setMask();
        
        uint32_t* data = new uint32_t[m_tileId*5]; // level, col, row, mask, pv id
        westTile.setTileMetadata(data);
        eastTile.setTileMetadata(data);
        unsigned char* p = (unsigned char*)data;
        std::string b64 = Utils::base64_encode(p, m_tileId*5*4);
        MetadataNode tilesMetadata3 = m_tileSetMetadata.add("tilesdata", b64);
        MetadataNode tilesMetadata2 = m_tileSetMetadata.add("tilesdatacount", m_tileId);
    }
    
    MetadataNode tilesMetadata = m_tileSetMetadata.addList("tiles");
    assert(tilesMetadata.valid());
}

void TileSet::done(PointTableRef table)
{
}


void TileSet::setStatisticsMetadata()
{
    MetadataNode outputNodes = m_tileSetMetadata.addList("statistics");
    assert(outputNodes.valid());

    MetadataNode inputRoot = m_tableMetadata.findChild("filters.stats");
    assert(inputRoot.valid());

    for (auto statisticNodes : inputRoot.children("statistic"))
    {
        assert(statisticNodes.valid());

        for (auto statisticNode : statisticNodes.children())
        {
            assert(statisticNode.valid());

            if (statisticNode.name().compare("name")==0) {
                std::string name = statisticNode.value();
                MetadataNode avg = statisticNodes.findChild("average");
                MetadataNode min = statisticNodes.findChild("minimum");
                MetadataNode max = statisticNodes.findChild("maximum");
                if (!avg.valid() || !min.valid() || !max.valid()) {
                    throw pdal_error("TileFilter: required stats metadata not found");
                }
                const std::string& avgS = avg.value();
                const std::string& minS = min.value();
                const std::string& maxS = max.value();

                MetadataNode outputNode = outputNodes.addList(name);
                outputNode.add("minimum", minS);
                outputNode.add("mean", avgS);
                outputNode.add("maximum", maxS);
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


void TileSet::setHeaderMetadata()
{
    MetadataNode node = m_tileSetMetadata.addList("header");
    assert(node.valid());

    node.add("maxLevel", m_maxLevel);
    node.add("numCols", 2);
    node.add("numRows", 1);
    node.add("minX", -180.0);
    node.add("minY", -90.0);
    node.add("maxX", 180.0);
    node.add("maxY", 90.0);
}


Tile::Tile(
        TileSet& tileSet,
        uint32_t level,
        uint32_t tx,
        uint32_t ty,
        const Rectangle& r) :
    m_tileSet(tileSet),
    m_level(level),
    m_tileX(tx),
    m_tileY(ty),
    m_children(NULL),
    m_skip(0),
    m_pointView(NULL),
    m_mask(0)
{
    m_rect.set(r);

    m_id = tileSet.newTileId();

    assert(m_level <= m_tileSet.getMaxLevel());

    //log()->get(LogLevel::Debug1) << "created tb (l=" << m_level
        //<< ", tx=" << m_tileX
        //<< ", ty=" << m_tileY
        //<< ") (slip" << m_skip
        //<< ")  --  w" << m_rect.west()
        //<< " s" << m_rect.south()
        //<< " e" << m_rect.east()
        //<< " n" << m_rect.north() << "\n";

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
    //log()->get(LogLevel::Debug1) << "level=" << m_level
        //<< "  skip=" << m_skip << "\n";
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
void Tile::setTileMetadata(uint32_t* data) const
{
    data[m_id*5+0] = m_level;
    data[m_id*5+1] = m_tileX;
    data[m_id*5+2] = m_tileY;
    data[m_id*5+3] = m_mask;
    data[m_id*5+4] = 0xffffffff;

    if (m_pointView)
    {
        data[m_id*5+4] = m_pointView->id();
    }

    if (m_children) {
        if (m_children[0]) m_children[0]->setTileMetadata(data);
        if (m_children[1]) m_children[1]->setTileMetadata(data);
        if (m_children[2]) m_children[2]->setTileMetadata(data);
        if (m_children[3]) m_children[3]->setTileMetadata(data);
    }
}


void Tile::setMask()
{
  // child mask
  m_mask = 0x0;
  if (m_children)
  {
      if (m_children[Rectangle::QuadrantSW]) m_mask += 1;
      if (m_children[Rectangle::QuadrantSE]) m_mask += 2;
      if (m_children[Rectangle::QuadrantNE]) m_mask += 4;
      if (m_children[Rectangle::QuadrantNW]) m_mask += 8;
  }

  if (m_children) {
      if (m_children[0]) m_children[0]->setMask();
      if (m_children[1]) m_children[1]->setMask();
      if (m_children[2]) m_children[2]->setMask();
      if (m_children[3]) m_children[3]->setMask();
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

    //log()->get(LogLevel::Debug5) << "-- -- " << pointNumber
        //<< " " << m_skip
        //<< " " << (pointNumber % m_skip == 0) << "\n";

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
    //log()->get(LogLevel::Debug5) << "which=" << q << "\n";

    Tile* child = m_children[q];
    if (child == NULL)
    {
        Rectangle r;
        m_rect.getRectangleOfQuadrant(q, r);
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

} // namespace tilercommon

} // namespace pdal
