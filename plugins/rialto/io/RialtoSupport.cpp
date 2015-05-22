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


//---------------------------------------------------------------------


TileTableInfo::TileTableInfo(const std::string& tileTableName,
                         MetadataNode tileTableNode,
                         PointLayoutPtr layout,
                         const std::string& datetime)
{
    m_name = tileTableName;

    m_datetime = datetime;

    MetadataNode headerNode = tileTableNode.findChild("header");
    assert(headerNode.valid());
    m_maxLevel = getMetadataU32(headerNode, "maxLevel");
    m_numDimensions = layout->dims().size();

    m_tmset_min_x = -180.0;
    m_tmset_min_y = -90.0;
    m_tmset_max_x = 180.0;
    m_tmset_max_y = 90.0;

    m_data_min_x = -189.0; // TODO
    m_data_min_y = -89.0;
    m_data_max_x = 179.0;
    m_data_max_y = 89.0;

    DimensionInfo::importVector(tileTableNode, layout, m_dimensions);
}


void TileTableInfo::set(const std::string& datetime,
                      const std::string& name,
                      uint32_t maxLevel,
                      uint32_t numDimensions,
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
    m_data_min_x = data_min_x;
    m_data_min_y = data_min_y;
    m_data_max_x = data_max_x;
    m_data_max_y = data_max_y;
    m_tmset_min_x = tmset_min_x;
    m_tmset_min_y = tmset_min_y;
    m_tmset_max_x = tmset_max_x;
    m_tmset_max_y = tmset_max_y;
}


DimensionInfo::DimensionInfo(const std::string& name,
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


void DimensionInfo::importVector(MetadataNode tileTableNode,
                                 PointLayoutPtr layout,
                                 std::vector<DimensionInfo>& infoList)
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

        DimensionInfo info(name, i, dataTypeName, description, minimum, mean, maximum);
        infoList.push_back(info);
        
        ++i;
    }
}


TileInfo::TileInfo(PointView* view,
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


void TileInfo::set(uint32_t level,
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


void WriterAssister::setTileTableName(const std::string& tileTableName)
{
    m_tileTableName = tileTableName;
}


void WriterAssister::ready(PointTableRef table)
{
    m_tileTableNode = table.metadata().findChild("filters.tiler");
    if (!m_tileTableNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler\" metadata not found");
    }

    time_t now;
    time(&now);
    char buf[sizeof("yyyy-mm-ddThh:mm:ss.sssZ")+1];
    // TODO: this produces "ss", not "ss.sss" as the gpkg spec implies is required
    strftime(buf, sizeof(buf), "%FT%TZ", gmtime(&now));
    std::string datetime(buf);
    writeHeader(m_tileTableName, m_tileTableNode, table.layout(), datetime);

    makePointViewMap();
}


void WriterAssister::write(const PointViewPtr viewPtr)
{
    // TODO: need to document/isolate this layout convention
    uint32_t idx = m_pointViewMap[viewPtr->id()];
    uint32_t level = m_tileMetadata[idx];
    uint32_t col = m_tileMetadata[idx+1];
    uint32_t row = m_tileMetadata[idx+2];
    uint32_t mask = m_tileMetadata[idx+3];
    uint32_t pvid = m_tileMetadata[idx+4];
    assert(pvid == 0xffffffff || pvid == (uint32_t)viewPtr->id());

    PointView* view = viewPtr.get();
    writeTile(m_tileTableName, view, level, col, row, mask);
}


void WriterAssister::makePointViewMap()
{
    // The tiler filter creates a metadata node for each point view,
    // and stores the point view id in that node.
    //
    // PDAL will be handing up point views, and we'll need to find
    // the corresponding metadata node for it -- so here will make
    // a reverse lookup, from point view id to metadata node.

    const MetadataNode tilesNode = m_tileTableNode.findChild("tiles");
    if (!tilesNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler/tiles\" metadata not found");
    }

    MetadataNode numTilesNode = m_tileTableNode.findChild("tilesdatacount");
    m_numTiles = boost::lexical_cast<uint32_t>(numTilesNode.value());

    const MetadataNode tilesNode2 = m_tileTableNode.findChild("tilesdata");
    std::string b64 = tilesNode2.value();
    std::vector<uint8_t> a = Utils::base64_decode(b64);

    m_tileMetadata = new uint32_t[m_numTiles*5];
    memcpy((unsigned char*)m_tileMetadata, a.data(), m_numTiles*5*4);
    for (uint32_t i=0; i<m_numTiles*5; i+=5)
    {
        uint32_t pv = m_tileMetadata[i+4];
        if (pv != 0xffffffff)
        {
            m_pointViewMap[pv] = i;
        }
    }
}


void WriterAssister::done()
{
    // write empty tiles
    
    const MetadataNode tilesNode = m_tileTableNode.findChild("tiles");
    const MetadataNodeList tileNodes = tilesNode.children();

    for (uint32_t i=0; i<m_numTiles*5; i+=5)
    {
        uint32_t level = m_tileMetadata[i];
        uint32_t col = m_tileMetadata[i+1];
        uint32_t row = m_tileMetadata[i+2];
        uint32_t mask = m_tileMetadata[i+3];
        uint32_t pvid = m_tileMetadata[i+4];

        if (pvid == 0xffffffff)
        {
            writeTile(m_tileTableName, NULL, level, col, row, mask);
        }
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
