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

#include "GeoPackageCommon.hpp"

namespace pdal
{
namespace rialto
{

    
static uint32_t getMetadataU32(const MetadataNode& parent, const std::string& name)
{
    const MetadataNode node = parent.findChild(name);
    if (!node.valid()) {
        std::ostringstream oss;
        oss << "RialtoWriter: required metadata item not found: " << name;
        throw pdal_error(oss.str());
    }
    uint32_t v = boost::lexical_cast<uint32_t>(node.value());
    return v;
}


static double getMetadataF64(const MetadataNode& parent, const std::string& name)
{
    const MetadataNode node = parent.findChild(name);
    if (!node.valid()) {
        std::ostringstream oss;
        oss << "RialtoWriter: required metadata item not found: " << name;
        throw pdal_error(oss.str());
    }
    double v = boost::lexical_cast<double>(node.value());
    return v;
}

static void extractStatistics(MetadataNode& tileTableNode, const std::string& dimName,
                                     double& minimum, double& mean, double& maximum)
{
    MetadataNode statisticNodes = tileTableNode.findChild("statistics");
    assert(statisticNodes.valid());

    MetadataNode dimNode = statisticNodes.findChild(dimName);
    if (!dimNode.valid())
    {
        std::ostringstream oss;
        oss << "RialtoWriter: statistics not found for dimension: " << dimName;
        throw pdal_error(oss.str());
    }

    minimum = getMetadataF64(dimNode, "minimum");
    mean = getMetadataF64(dimNode, "mean");
    maximum = getMetadataF64(dimNode, "maximum");
}


GpkgMatrixSet::GpkgMatrixSet(const std::string& tileTableName,
                         MetadataNode tileTableNode,
                         PointLayoutPtr layout,
                         const std::string& datetime,
                         const SpatialReference& srs)
{
    m_name = tileTableName;

    m_datetime = datetime;

    MetadataNode headerNode = tileTableNode.findChild("header");
    assert(headerNode.valid());
    m_maxLevel = getMetadataU32(headerNode, "maxLevel");
    m_numDimensions = layout->dims().size();

    m_wkt = srs.getWKT(SpatialReference::eCompoundOK);
    
    m_tmset_min_x = -180.0;
    m_tmset_min_y = -90.0;
    m_tmset_max_x = 180.0;
    m_tmset_max_y = 90.0;

    double statMinX, statMeanX, statMaxX;
    double statMinY, statMeanY, statMaxY;
    extractStatistics(tileTableNode, "X", statMinX, statMeanX, statMaxX);
    extractStatistics(tileTableNode, "Y", statMinY, statMeanY, statMaxY);

    m_data_min_x = statMinX; // TODO
    m_data_min_y = statMinY;
    m_data_max_x = statMaxX;
    m_data_max_y = statMaxY;

    GpkgDimension::importVector(tileTableNode, layout, m_dimensions);
}


void GpkgMatrixSet::set(const std::string& datetime,
                      const std::string& name,
                      uint32_t maxLevel,
                      uint32_t numDimensions,
                      const std::string& wkt,
                      double data_min_x,
                      double data_min_y,
                      double data_max_x,
                      double data_max_y,
                      double tmset_min_x,
                      double tmset_min_y,
                      double tmset_max_x,
                      double tmset_max_y)
{
    m_datetime = datetime;
    m_name = name;
    m_maxLevel = maxLevel;
    m_numDimensions = numDimensions;
    m_wkt = wkt;
    m_data_min_x = data_min_x;
    m_data_min_y = data_min_y;
    m_data_max_x = data_max_x;
    m_data_max_y = data_max_y;
    m_tmset_min_x = tmset_min_x;
    m_tmset_min_y = tmset_min_y;
    m_tmset_max_x = tmset_max_x;
    m_tmset_max_y = tmset_max_y;
}


GpkgDimension::GpkgDimension(const std::string& name,
                             uint32_t position,
                             const std::string& dataType,
                             const std::string& description,
                             double minimum,
                             double mean,
                             double maximum) :
    m_name(name),
    m_position(position),
    m_dataType(dataType),
    m_description(description),
    m_minimum(minimum),
    m_mean(mean),
    m_maximum(maximum)
{ }       


void GpkgDimension::importVector(MetadataNode tileTableNode,
                                 PointLayoutPtr layout,
                                 std::vector<GpkgDimension>& infoList)
{
    const uint32_t numDims = layout->dims().size();

    infoList.clear();

    //log()->get(LogLevel::Debug1) << "num dims: " << infoList.size() << std::endl;

    size_t i = 0;
    for (const auto& dim : layout->dims())
    {
        const std::string name = Dimension::name(dim);
        const std::string description = Dimension::description(dim);
        const std::string& dataTypeName = Dimension::interpretationName(layout->dimType(dim));

        double minimum, mean, maximum;
        extractStatistics(tileTableNode, name, minimum, mean, maximum);

        GpkgDimension info(name, i, dataTypeName, description, minimum, mean, maximum);
        infoList.push_back(info);
        
        ++i;
    }
}


GpkgTile::GpkgTile(PointView* view,
                   uint32_t level, uint32_t column, uint32_t row, uint32_t mask) :
    m_level(level),
    m_column(column),
    m_row(row),
    m_numPoints(0),
    m_mask(mask)
{
    m_patch.clear();

    if (view)
    {
        m_numPoints = view->size();
        m_patch.importFromPV(*view);
    }
}


void GpkgTile::set(uint32_t level,
                   uint32_t column,
                   uint32_t row,
                   uint32_t numPoints,
                   uint32_t mask)
{
    m_level = level;
    m_column = column;
    m_row = row;
    m_numPoints = numPoints;
    m_mask = mask;
}



} // namespace rialto
} // namespace pdal
