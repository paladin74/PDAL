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

#include "RialtoWriter.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_error.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include <cstdint>

namespace pdal
{

namespace
{
 // anonymous namespace
}

void RialtoWriter::serializeToTileSetInfo(const std::string& tileSetName,
                                          MetadataNode tileSetNode,
                                          PointLayoutPtr layout,
                                          RialtoDb::TileSetInfo& tileSetInfo)
{
    tileSetInfo.nam = tileSetName;

    MetadataNode headerNode = tileSetNode.findChild("header");
    assert(headerNode.valid());
    tileSetInfo.maxLevel = RialtoWriter::getMetadataU32(headerNode, "maxLevel");    
    tileSetInfo.numDimensions = layout->dims().size();
    
    tileSetInfo.tmset_min_x = -180.0;
    tileSetInfo.tmset_min_y = -90.0;
    tileSetInfo.tmset_max_x = 180.0;
    tileSetInfo.tmset_max_y = 90.0;

    tileSetInfo.data_min_x = -189.0; // TODO
    tileSetInfo.data_min_y = -89.0;
    tileSetInfo.data_max_x = 179.0;
    tileSetInfo.data_max_y = 89.0;

    serializeToDimensionInfo(tileSetNode, layout, tileSetInfo.dimensions);    
}


void RialtoWriter::serializeToDimensionInfo(MetadataNode tileSetNode,
                                            PointLayoutPtr layout,
                                            std::vector<RialtoDb::DimensionInfo>& infoList)
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
        RialtoWriter::extractStatistics(tileSetNode, name, minimum, mean, maximum);

        RialtoDb::DimensionInfo& info = infoList[i];
        info.name = name;
        info.dataType = dataTypeName;
        info.position = i;
        info.minimum = minimum;
        info.mean = mean;
        info.maximum = maximum;

        ++i;
    }
}


void RialtoWriter::serializeToPatch(const PointView& view, Patch& patch)
{        
    const uint32_t pointSize = view.pointSize();
    const uint32_t numPoints = view.size();
    const uint32_t buflen = pointSize * numPoints;

    patch.buf.resize(buflen);
    
    char* buf = (char*)(&patch.buf[0]);
    char* p = buf;
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

    assert(patch.buf.size() == buflen);
}


void RialtoWriter::serializeToTileInfo(uint32_t tileSetId, PointView* view, RialtoDb::TileInfo& tileInfo,
    uint32_t level, uint32_t col, uint32_t row, uint32_t mask)
{
    tileInfo.tileSetId = tileSetId;
    
    tileInfo.level = level;
    tileInfo.column = col;
    tileInfo.row = row;
    tileInfo.mask = mask;

    tileInfo.numPoints = 0;
    if (view)
    {
        tileInfo.numPoints = view->size();
    }

    Patch& patch = tileInfo.patch;
    if (!view)
    {
        patch.buf.clear();
    } else {
        serializeToPatch(*view, patch);
    }
}


void RialtoWriter::extractStatistics(MetadataNode& tileSetNode, const std::string& dimName,
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


uint32_t RialtoWriter::getMetadataU32(const MetadataNode& parent, const std::string& name)
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


double RialtoWriter::getMetadataF64(const MetadataNode& parent, const std::string& name)
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


void RialtoWriter::ready(PointTableRef table)
{
    log()->get(LogLevel::Debug) << "RialtoWriter::ready()" << std::endl;

    m_table = &table;

    localStart();

    MetadataNode tileSetNode = m_table->metadata().findChild("filters.tiler");
    if (!tileSetNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler\" metadata not found");
    }

    m_tileSetId = writeHeader(m_tileSetName, tileSetNode, m_table->layout());

    makePointViewMap(tileSetNode);
}


// write out the tile with this pointview
void RialtoWriter::write(const PointViewPtr viewPtr)
{
    //log()->get(LogLevel::Debug1) << "RialtoWriter::write()" << std::endl;
    
    PointView* view = viewPtr.get();

    uint32_t idx = m_pointViewMap2[viewPtr->id()];
    uint32_t level = m_tileMetadata[idx];
    uint32_t col = m_tileMetadata[idx+1];
    uint32_t row = m_tileMetadata[idx+2];
    uint32_t mask = m_tileMetadata[idx+3];
    uint32_t pvid = m_tileMetadata[idx+4];
    assert(pvid == 0xffffffff || pvid == (uint32_t)viewPtr->id());

    writeTile(m_tileSetId, m_tileSetName, view, level, col, row, mask);
}


// write out all the remaining tiles: those without point views
void RialtoWriter::done(PointTableRef table)
{
    writeEmptyTiles();

    localFinish();
}


void RialtoWriter::makePointViewMap(MetadataNode tileSetNode)
{
    // The tiler filter creates a metadata node for each point view,
    // and stores the point view id in that node.
    // 
    // PDAL will be handing up point views, and we'll need to find
    // the corresponding metadata node for it -- so here will make
    // a reverse lookup, from point view id to metadata node.

    const MetadataNode tilesNode = tileSetNode.findChild("tiles");
    if (!tilesNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler/tiles\" metadata not found");
    }
    
    MetadataNode numTilesNode = tileSetNode.findChild("tilesdatacount");
    m_numTiles = boost::lexical_cast<uint32_t>(numTilesNode.value());

    const MetadataNode tilesNode2 = tileSetNode.findChild("tilesdata");
    std::string b64 = tilesNode2.value();
    std::vector<uint8_t> a = Utils::base64_decode(b64);

    m_tileMetadata = new uint32_t[m_numTiles*5];
    memcpy((unsigned char*)m_tileMetadata, a.data(), m_numTiles*5*4);
    for (uint32_t i=0; i<m_numTiles*5; i+=5)
    {
        uint32_t pv = m_tileMetadata[i+4];
        if (pv != 0xffffffff)
        {
            m_pointViewMap2[pv] = i;
        }
    }
}


void RialtoWriter::writeEmptyTiles()
{
    const MetadataNode tileSetNode = m_table->metadata().findChild("filters.tiler");
    const MetadataNode tilesNode = tileSetNode.findChild("tiles");
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
            writeTile(m_tileSetId, m_tileSetName, NULL, level, col, row, mask);
        }
    }
}


// TODO: this is a dup of RialtoDb::castPatchAsBuffer
void RialtoWriter::castPatchAsBuffer(const Patch& patch, unsigned char*& buf, uint32_t& bufLen)
{
    buf = NULL;
    bufLen = patch.buf.size();    
    if (bufLen) {
        buf = (unsigned char*)&patch.buf[0];
    }
}

} // namespace pdal
