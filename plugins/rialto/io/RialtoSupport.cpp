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

#include "RialtoSupport.hpp"

namespace pdal
{
namespace rialto
{

    
//---------------------------------------------------------------------

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

static void extractStatistics(MetadataNode& tileSetNode, const std::string& dimName,
                                     double& minimum, double& mean, double& maximum)
{
    MetadataNode statisticNodes = tileSetNode.findChild("statistics");
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


//---------------------------------------------------------------------


TileSetInfo::TileSetInfo(const std::string& tileSetName,
                         MetadataNode tileSetNode,
                         PointLayoutPtr layout)
{
    name = tileSetName;

    time_t now;
    time(&now);
    char buf[sizeof("yyyy-mm-ddThh:mm:ss.sssZ")+1];
    // TODO: this produces "ss", not "ss.sss" as the gpkg spec implies is required
    strftime(buf, sizeof(buf), "%FT%TZ", gmtime(&now));
    datetime = buf;
    
    MetadataNode headerNode = tileSetNode.findChild("header");
    assert(headerNode.valid());
    maxLevel = getMetadataU32(headerNode, "maxLevel");    
    numDimensions = layout->dims().size();
    
    tmset_min_x = -180.0;
    tmset_min_y = -90.0;
    tmset_max_x = 180.0;
    tmset_max_y = 90.0;

    data_min_x = -189.0; // TODO
    data_min_y = -89.0;
    data_max_x = 179.0;
    data_max_y = 89.0;

    DimensionInfo::import(tileSetNode, layout, dimensions);
}


void DimensionInfo::import(MetadataNode tileSetNode,
                                            PointLayoutPtr layout,
                                            std::vector<DimensionInfo>& infoList)
{    
    const uint32_t numDims = layout->dims().size();
    
    infoList.clear();
    infoList.resize(numDims);

    //log()->get(LogLevel::Debug1) << "num dims: " << infoList.size() << std::endl;

    size_t i = 0;
    for (const auto& dim : layout->dims())
    {
        const std::string name = Dimension::name(dim);
        const std::string& dataTypeName = Dimension::interpretationName(layout->dimType(dim));

        double minimum, mean, maximum;
        extractStatistics(tileSetNode, name, minimum, mean, maximum);

        DimensionInfo& info = infoList[i];
        info.name = name;
        info.dataType = dataTypeName;
        info.position = i;
        info.minimum = minimum;
        info.mean = mean;
        info.maximum = maximum;

        ++i;
    }
}


TileInfo::TileInfo(PointView* view,
                   uint32_t l, uint32_t c, uint32_t r, uint32_t m) :
    level(l),
    column(c),
    row(r),
    numPoints(0),
    mask(m)
{
    patch.clear();

    if (view)
    {
        numPoints = view->size();
        patch.importFromPV(*view);
    }
}


//---------------------------------------------------------------------


uint32_t MyPatch::size() const
{
     return m_vector.size(); 
}


void MyPatch::clear()
{
    m_vector.clear();
}


bool MyPatch::isEmpty() const
{
    return m_vector.size()==0;
}

const std::vector<unsigned char>& MyPatch::getVector() const
{
    return m_vector;
}
    

const unsigned char* MyPatch::getPointer() const
{
    if (isEmpty()) return NULL;
    return (const unsigned char*)&m_vector[0];
}

void MyPatch::importFromVector(const std::vector<uint8_t>& vec)
{
    m_vector = vec;
}

void MyPatch::importFromPV(const PointView& view)
{
    const uint32_t pointSize = view.pointSize();
    const uint32_t numPoints = view.size();
    const uint32_t buflen = pointSize * numPoints;

    m_vector.resize(buflen);
    
    char* p = (char*)(&m_vector[0]);
    const DimTypeList& dtl = view.dimTypes();

    uint32_t numBytes = 0;
    for (auto d: dtl)
    {
        numBytes += Dimension::size(d.m_type);
    }

    for (size_t i=0; i<numPoints; ++i)
    {
        view.getPackedPoint(dtl, i, p);
        p += numBytes;
    }

    assert(m_vector.size() == buflen);
}

// does an append to the PV (does not start at index 0)
void MyPatch::exportToPV(size_t numPoints, PointViewPtr view) const
{
    PointId idx = view->size();
    const uint32_t pointSize = view->pointSize();

    const char* p = (const char*)(&m_vector[0]);
    const DimTypeList& dtl = view->dimTypes();
    for (size_t i=0; i<numPoints; ++i)
    {
        view->setPackedPoint(dtl, idx, p);
        p += pointSize;
        ++idx;
    }
}


//---------------------------------------------------------------------


RialtoEvent::RialtoEvent(const std::string& name) :
  m_name(name),
  m_count(0),
  m_millis(0.0),
  m_start(0)
{}


RialtoEvent::~RialtoEvent()
{
    assert(m_start == 0);
}


void RialtoEvent::start()
{
    assert(m_start == 0);
    m_start = timerStart();
}


void RialtoEvent::stop()
{
    assert(m_start != 0);
    ++m_count;
    m_millis += timerStop(m_start);
    m_start = 0;
}


void RialtoEvent::dump() const
{
    if (m_count)
    {
        printf("%s:  total=%.1fms  average=%.1fms  (%u events)\n",
               m_name.c_str(),
               m_millis,
               m_millis/(double)m_count,
               m_count);
    }
    else
    {
        printf("%s: -\n", m_name.c_str());
    }
}


clock_t RialtoEvent::timerStart()
{
     return std::clock();
}


double RialtoEvent::timerStop(clock_t start)
{
    clock_t stop = std::clock();
    const double secs = (double)(stop - start) / (double)CLOCKS_PER_SEC;
    return secs * 1000.0;
}


} // namespace rialto
} // namespace pdal
