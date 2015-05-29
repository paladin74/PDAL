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

#pragma once

#include <pdal/pdal.hpp>

namespace pdal
{
namespace rialto
{


class RialtoWriterAssister
{
public:
    void setParameters(const std::string& matrixSetName,
                       uint32_t numColsAtL0,
                       uint32_t numRowsAtL0,
                       const std::string& description,
                       const std::string& timestamp);
    
    void write(const PointViewPtr viewPtr);
    void ready(PointTableRef table, const SpatialReference& srs);
    void done();

protected:
    virtual void writeHeader(MetadataNode tileTableNode,
                             PointLayoutPtr layout,
                             const SpatialReference& srs,
                             const std::string& lasMetadata)=0;
                             
    virtual void writeTile(PointView*,
                           uint32_t level, uint32_t col, uint32_t row,
                           uint32_t mask)=0;

    virtual void writeTiles_begin() {}
    virtual void writeTiles_end() {}
    
    std::string m_matrixSetName;
    uint32_t m_numColsAtL0;
    uint32_t m_numRowsAtL0;
    std::string m_description;
    std::string m_timestamp;

private:    
    void makePointViewMap();

    std::map<uint32_t, uint32_t> m_pointViewMap; // PV id to array index
    uint32_t* m_tileMetadata;
    uint32_t m_numTiles;
    
    MetadataNode m_tileTableNode;
};



} // namespace rialto
} // namespace pdal
