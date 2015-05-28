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

#include "RialtoWriterAssister.hpp"

namespace pdal
{
namespace rialto
{


void RialtoWriterAssister::setParameters(const std::string& matrixSetName,
                                         uint32_t numColsAtL0,
                                         uint32_t numRowsAtL0,
                                         const std::string& description,
                                         const std::string& timestamp)
{
    m_matrixSetName = matrixSetName;
    m_numColsAtL0 = numColsAtL0;
    m_numRowsAtL0 = numRowsAtL0;
    m_description = description;
    m_timestamp = timestamp;
}


void RialtoWriterAssister::ready(PointTableRef table, const SpatialReference& srs)
{
    m_tileTableNode = table.metadata().findChild("filters.tiler");
    if (!m_tileTableNode.valid()) {
        throw pdal_error("RialtoWriter: \"filters.tiler\" metadata not found");
    }

    writeHeader(m_tileTableNode, table.layout(), srs);

    makePointViewMap();
    
    writeTiles_begin();
}


void RialtoWriterAssister::write(const PointViewPtr viewPtr)
{
    uint32_t idx = m_pointViewMap[viewPtr->id()];
    uint32_t level = m_tileMetadata[idx];
    uint32_t col = m_tileMetadata[idx+1];
    uint32_t row = m_tileMetadata[idx+2];
    uint32_t mask = m_tileMetadata[idx+3];
    uint32_t pvid = m_tileMetadata[idx+4];
    assert(pvid == 0xffffffff || pvid == (uint32_t)viewPtr->id());

    PointView* view = viewPtr.get();
    writeTile(view, level, col, row, mask);
}


void RialtoWriterAssister::makePointViewMap()
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

    // for details on this blob of five uint32s, see TileSet::run()
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


void RialtoWriterAssister::done()
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
            writeTile(NULL, level, col, row, mask);
        }
    }
    
    writeTiles_end();
}


//---------------------------------------------------------------------


} // namespace rialto
} // namespace pdal
