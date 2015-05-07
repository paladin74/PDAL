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

#include "RialtoDbWriter.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_error.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include "RialtoDb.hpp"

#include <cstdint>


namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.rialtodb",
    "Rialto DB Writer",
    "http://pdal.io/stages/writers.rialtodb.html" );

CREATE_SHARED_PLUGIN(1, 0, RialtoDbWriter, Writer, s_info)

namespace
{
    static void serializeToTileSetInfo(MetadataNode tileSetNode,
                                       PointLayoutPtr layout,
                                       RialtoDb::TileSetInfo& tileSetInfo)
    {
        tileSetInfo.name = "a.las"; // TODO

        MetadataNode headerNode = tileSetNode.findChild("header");
        assert(headerNode.valid());
        tileSetInfo.maxLevel = RialtoDbWriter::getMetadataU32(headerNode, "maxLevel");

        tileSetInfo.numCols = RialtoDbWriter::getMetadataU32(headerNode, "numCols");
        tileSetInfo.numRows = RialtoDbWriter::getMetadataU32(headerNode, "numRows");
        assert(tileSetInfo.numCols == 2 && tileSetInfo.numRows == 1);

        tileSetInfo.minx = RialtoDbWriter::getMetadataF64(headerNode, "minX");
        tileSetInfo.miny = RialtoDbWriter::getMetadataF64(headerNode, "minY");
        tileSetInfo.maxx = RialtoDbWriter::getMetadataF64(headerNode, "maxX");
        tileSetInfo.maxy = RialtoDbWriter::getMetadataF64(headerNode, "maxY");
        assert(tileSetInfo.minx==-180.0 && tileSetInfo.miny==-90.0 && tileSetInfo.maxx==180.0 && tileSetInfo.maxy==90.0);
        
        tileSetInfo.numDimensions = layout->dims().size();
    }

    static void serializeToDimensionInfo(MetadataNode tileSetNode,
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
            const Dimension::Type::Enum dataType = layout->dimType(dim);

            double minimum, mean, maximum;
            RialtoDbWriter::extractStatistics(tileSetNode, name, minimum, mean, maximum);

            RialtoDb::DimensionInfo& info = infoList[i];
            info.name = name;
            info.position = i;
            info.dataType = (RialtoDb::DataType)(uint32_t)dataType; // TODO: enum-to-enum
            info.minimum = minimum;
            info.mean = mean;
            info.maximum = maximum;

            ++i;
        }
    }


    static void serializeToPatch(PointView* view, Patch& patch)
    {
        if (!view)
        {
            patch.buf.clear();
        } else {
            size_t len = 0;
            unsigned char* buf = RialtoWriter::createBlob(view, len);  // TODO
            patch.putBytes(buf, len);
            delete[] buf; // TODO
        }
    }


    static void serializeToTile(MetadataNode tileNode, PointView* view, RialtoDb::TileInfo& tileInfo)
    {
        tileInfo.tileSetId = 0; // not used in writing
        
        tileInfo.level = RialtoWriter::getMetadataU32(tileNode, "level");
        tileInfo.x = RialtoWriter::getMetadataU32(tileNode, "tileX");
        tileInfo.y = RialtoWriter::getMetadataU32(tileNode, "tileY");
        //const uint32_t mask = getMetadataU32(tileNode, "mask");

        //log()->get(LogLevel::Debug) << "RialtoDbWriter::writeTile for "
        //    << tileInfo.level << "," << info.x << "," << info.y << " "
        //    << (view==0 ? "no" : "yes")
        //    << std::endl;

        Patch& patch = tileInfo.patch;
        serializeToPatch(view, patch);
    }
} // anonymous namespace



void RialtoDbWriter::writeHeader(MetadataNode tileSetNode,
                                 PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::writeHeader()" << std::endl;

    RialtoDb::TileSetInfo tileSetInfo;
    serializeToTileSetInfo(tileSetNode, layout, tileSetInfo);

    std::vector<RialtoDb::DimensionInfo> dimsInfo;
    serializeToDimensionInfo(tileSetNode, layout, dimsInfo);
    
    const uint32_t id = m_rialtoDb->addTileSet(tileSetInfo);

    m_rialtoDb->addDimensions(id, dimsInfo);
}


void RialtoDbWriter::writeTile(MetadataNode tileNode, PointView* view)
{
    log()->get(LogLevel::Debug1) << "RialtoDbWriter::writeTile()" << std::endl;

    RialtoDb::TileInfo tileInfo;
    serializeToTile(tileNode, view, tileInfo);

    if (tileInfo.patch.buf.size())
    {
        uint32_t id = m_rialtoDb->addTile(tileInfo);
    }
}


std::string RialtoDbWriter::getName() const
{
    return s_info.name;
}


void RialtoDbWriter::processOptions(const Options& options)
{
    // we treat the target "filename" as the database name,
    // so we'll use a differently named variable to make it clear
    m_connection = m_filename;
}


Options RialtoDbWriter::getDefaultOptions()
{
    Options options;
    options.add("verbose", 10);
    return options;
}


void RialtoDbWriter::localStart()
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::localStart()" << std::endl;

    // pdal writers always clobber their output file, so we follow
    // the same convention here -- even though we're dealing with
    // an output "database" instead of an output "file"

    FileUtils::deleteFile(m_connection);

    m_rialtoDb = new RialtoDb(m_connection);
    m_rialtoDb->create();
}


void RialtoDbWriter::localFinish()
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::localFinish()" << std::endl;

    m_rialtoDb->close();
}


} // namespace pdal
