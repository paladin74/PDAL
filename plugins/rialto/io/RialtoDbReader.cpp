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
#include "GeoPackageReader.hpp"
#include "GeoPackageCommon.hpp"
#include <../filters/tiler/TilerCommon.hpp>

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
        m_db = new GeoPackageReader(m_filename, log());
        m_db->open();

        m_matrixSet = std::unique_ptr<GpkgMatrixSet>(new GpkgMatrixSet());

        if (m_matrixSetName == "")
        {
            std::vector<std::string> names;
            m_db->readMatrixSetNames(names);
            if (names.size() == 0)
            {
                throw pdal_error("geopackage has no tile matrix sets");
            }
            if (names.size() > 1)
            {
                throw pdal_error("tile matrix set name not specified in options list");
            }
            m_matrixSetName = names[0];
        }

        m_db->readMatrixSet(m_matrixSetName, *m_matrixSet);
        
        const SpatialReference srs(m_matrixSet->getWkt());
        setSpatialReference(srs);
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

    if (!m_db)
    {
        // you can't change the filename or dataset name once we've opened the DB
        m_filename = options.getValueOrThrow<std::string>("filename");
        m_matrixSetName = options.getValueOrDefault<std::string>("name", "");
    }
    
    m_queryBox = options.getValueOrDefault<BOX3D>("bounds", BOX3D());
    m_queryLevel = options.getValueOrDefault<uint32_t>("level", 0xffff);

    log()->get(LogLevel::Debug) << "process options: bounds=" << m_queryBox << std::endl;
}


void RialtoDbReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::addDimensions()" << std::endl;

    m_db->setupLayout(*m_matrixSet, layout);
}


void RialtoDbReader::ready(PointTableRef table)
{
    log()->get(LogLevel::Debug) << "RialtoDbReader::ready()" << std::endl;
}


void RialtoDbReader::setQueryParams()
{
    if (m_queryBox.empty())
    {
        m_queryBox.minx = m_matrixSet->getDataMinX();
        m_queryBox.miny = m_matrixSet->getDataMinY();
        m_queryBox.maxx = m_matrixSet->getDataMaxX();
        m_queryBox.maxy = m_matrixSet->getDataMaxY();
    }
            
    if (m_queryLevel == 0xffff)
    {
        m_queryLevel = m_matrixSet->getMaxLevel();
    }
    else if (m_queryLevel > m_matrixSet->getMaxLevel())
    {
        throw pdal_error("Zoom level set higher than data set allows");
    }
}


point_count_t RialtoDbReader::read(PointViewPtr view, point_count_t /*not used*/)
{
    // TODO: okay to ignore point count parameter?
    
    log()->get(LogLevel::Debug) << "RialtoDbReader::read()" << std::endl;

    const tilercommon::TileMatrixMath tmm(m_matrixSet->getTmsetMinX(), m_matrixSet->getTmsetMinY(),
                                          m_matrixSet->getTmsetMaxX(), m_matrixSet->getTmsetMaxY(),
                                          m_matrixSet->getNumColsAtL0(), m_matrixSet->getNumRowsAtL0());

    setQueryParams();
    
    const double qMinX = m_queryBox.minx;
    const double qMinY = m_queryBox.miny;
    const double qMaxX = m_queryBox.maxx;
    const double qMaxY = m_queryBox.maxy;

    const uint32_t level = m_queryLevel;

    m_db->queryForTiles_begin(m_matrixSetName, qMinX, qMinY, qMaxX, qMaxY, level);

    GpkgTile info;

    do {
        bool ok = m_db->queryForTiles_step(info);
        if (!ok) break;

        // if this tile is entirely inside the query box, then
        // we won't need to check each point
        double tileMinX, tileMinY, tileMaxX, tileMaxY;
        tmm.getTileBounds(info.getColumn(), info.getRow(), info.getLevel(),
                          tileMinX, tileMinY, tileMaxX, tileMaxY);
        const bool allPointsGood =
            tmm.rectContainsRect(qMinX, qMinY, qMaxX, qMaxY,
                                 tileMinX, tileMinY, tileMaxX, tileMaxY);
        
        log()->get(LogLevel::Debug) << "  got some points: " << info.getNumPoints() << std::endl;

        PointViewPtr tempView = view->makeNew();

        info.getPatch().exportToPV(info.getNumPoints(), tempView);

        for (uint32_t i=0; i<tempView->size(); i++) {
            const double x = tempView->getFieldAs<double>(Dimension::Id::X, i);
            const double y = tempView->getFieldAs<double>(Dimension::Id::Y, i);
            if (allPointsGood || (x >= qMinX && x <= qMaxX && y >= qMinY && y <= qMaxY))
            {
                view->appendPoint(*tempView, i);
            }
        }

        log()->get(LogLevel::Debug) << "  view now has this many: " << view->size() << std::endl;
    } while (m_db->queryForTiles_next());
  
    return view->size();
}

} // namespace rialto

} // namespace pdal
