/******************************************************************************
* Copyright (c) 2014-2015, RadiantBlue Technologies, Inc.
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
    static void fillBufferWithPoint(const PointView* view, const PointId& idx, char* buf)
    {
        char* p = buf;
        char* q = p;

        for (const auto& dim : view->dims())
        {
            view->getRawField(dim, idx, q);
            q += view->dimSize(dim);
        }
    }

    static void fillBufferWithPointView(const PointView* view, char* buf)
    {
        for (size_t i=0; i<view->size(); ++i)
        {
            const PointId idx = i;
            fillBufferWithPoint(view, idx, buf);
        }
    
    }
    
} // anonymous namespace


// caller responisble for deleting the buffer
char* RialtoWriter::createBlob(PointView* view, size_t& buflen)
{
    const uint32_t pointSize = view->pointSize();
    const uint32_t numPoints = view->size();
    buflen = pointSize * numPoints;
    char* buf = new char[buflen];

    fillBufferWithPointView(view, buf);

    return buf;
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
    m_table = &table;

    localStart();

    MetadataNode tileSetNode = m_table->metadata().findChild("filters.tiler");
    if (!tileSetNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler\" metadata not found");
    }

    writeHeader(tileSetNode, m_table->layout());

    makePointViewMap(tileSetNode);
}


// write out the tile with this pointview
void RialtoWriter::write(const PointViewPtr viewPtr)
{
    MetadataNode tileNode = m_pointViewMap[viewPtr->id()];
    assert(tileNode.valid());

    PointView* view = viewPtr.get();
    writeTile(tileNode, view);
}


// write out all the remaining tiles: those without point views
void RialtoWriter::done(PointTableRef table)
{
    writeEmptyTiles();

    localFinish();
}


void RialtoWriter::makePointViewMap(MetadataNode tileSetNode)
{
    // the metadata nodes are listed by tile id, not by point view id,
    // so we need make to make a map from point view id to metadata node
    const MetadataNode tilesNode = tileSetNode.findChild("tiles");
    if (!tilesNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler/tiles\" metadata not found");
    }
    const MetadataNodeList tileNodes = tilesNode.children();
    for (auto node: tileNodes)
    {
        MetadataNode n = node.findChild("pointView");
        if (n.valid()) {
          const uint32_t viewId = boost::lexical_cast<uint32_t>(n.value());
          m_pointViewMap[viewId] = node;
        }
    }
}


void RialtoWriter::writeEmptyTiles()
{
    const MetadataNode tileSetNode = m_table->metadata().findChild("filters.tiler");
    const MetadataNode tilesNode = tileSetNode.findChild("tiles");
    const MetadataNodeList tileNodes = tilesNode.children();

    for (auto iter = tileNodes.begin(); iter != tileNodes.end(); ++iter)
    {
        MetadataNode tileNode = *iter;
        const MetadataNode nodeP = tileNode.findChild("pointView");
        if (!nodeP.valid()) {
          writeTile(tileNode, NULL);
        }
    }
}


} // namespace pdal
