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
} // anonymous namespace


void RialtoDbWriter::writeHeader(MetadataNode tileSetNode,
                                   PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::writeHeader()" << std::endl;

    RialtoDb::TileSetInfo data;
    data.name = "a.las";

    MetadataNode headerNode = tileSetNode.findChild("header");
    assert(headerNode.valid());
    data.maxLevel = getMetadataU32(headerNode, "maxLevel");

    data.numCols = getMetadataU32(headerNode, "numCols");
    data.numRows = getMetadataU32(headerNode, "numRows");
    assert(data.numCols == 2 && data.numRows == 1);
    
    data.minx = getMetadataF64(headerNode, "minX");
    data.miny = getMetadataF64(headerNode, "minY");
    data.maxx = getMetadataF64(headerNode, "maxX");
    data.maxy = getMetadataF64(headerNode, "maxY");
    assert(data.minx==-180.0 && data.miny==-90.0 && data.maxx==180.0 && data.maxy==90.0);

    data.numDimensions = 0;

    uint32_t id = m_rialtoDb->addTileSet(data);

/*
    fprintf(fp, "    \"version\": 4,\n");

    fprintf(fp, "    \"dimensions\": [\n");

    const size_t numDims = layout->dims().size();
    size_t i = 0;
    for (const auto& dim : layout->dims())
    {
        const Dimension::Type::Enum dataType = layout->dimType(dim);
        const std::string& dataTypeName = Dimension::interpretationName(dataType);
        const std::string& name = Dimension::name(dim);

        double minimum, mean, maximum;
        extractStatistics(tileSetNode, name, minimum, mean, maximum);

        fprintf(fp, "        {\n");
        fprintf(fp, "            \"datatype\": \"%s\",\n", dataTypeName.c_str());
        fprintf(fp, "            \"name\": \"%s\",\n", name.c_str());
        fprintf(fp, "            \"minimum\": %f,\n", minimum);
        fprintf(fp, "            \"mean\": %f,\n", mean);
        fprintf(fp, "            \"maximum\": %f\n", maximum);
        fprintf(fp, "        }%s\n", i++==numDims-1 ? "" : ",");
    }
*/
}

void RialtoDbWriter::writeTile(MetadataNode tileNode, PointView* view)
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::writeTile()" << std::endl;

    RialtoDb::TileInfo data;
    data.tileSetId = 0;
    data.level = getMetadataU32(tileNode, "level");
    data.x = getMetadataU32(tileNode, "tileX");
    data.y = getMetadataU32(tileNode, "tileY");
    const uint32_t mask = getMetadataU32(tileNode, "mask");

    uint32_t id = m_rialtoDb->addTile(data, NULL);

/*
    if (view)
    {
        size_t bufsiz;
        char* buf = createBlob(view, bufsiz);
        fwrite(buf, bufsiz, 1, fp);
    }

    uint8_t mask8 = mask;
    fwrite(&mask8, 1, 1, fp);
*/

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
    m_rialtoDb->open(true);
}


void RialtoDbWriter::localFinish()
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::localFinish()" << std::endl;

    m_rialtoDb->close();
}


} // namespace pdal
