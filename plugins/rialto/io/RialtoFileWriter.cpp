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

#include "RialtoFileWriter.hpp"

#include "RialtoDb.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.rialtofile",
    "Rialto File Writer",
    "http://pdal.io/stages/writers.rialtofile.html" );

CREATE_SHARED_PLUGIN(1, 0, rialto::RialtoFileWriter, Writer, s_info)

namespace rialto
{


void RialtoFileWriter::writeHeader(const std::string& tileSetName,
                                   MetadataNode tileSetNode,
                                   PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoFileWriter::writeHeader()" << std::endl;

    const TileSetInfo tileSetInfo(tileSetName, tileSetNode, layout);
    
    const std::string filename(m_directory + "/header.json");
    FILE* fp = fopen(filename.c_str(), "wt");

    fprintf(fp, "{\n");
    fprintf(fp, "    \"version\": 4,\n");
    fprintf(fp, "    \"maxLevel\": %d,\n", tileSetInfo.maxLevel);
    fprintf(fp, "    \"dimensions\": [\n");

    
    std::vector<DimensionInfo> dimsInfo;
    DimensionInfo::import(tileSetNode, layout, dimsInfo);

    const size_t numDims = dimsInfo.size();
    size_t i = 0;
    for (auto& dimInfo : dimsInfo)
    {
        fprintf(fp, "        {\n");
        fprintf(fp, "            \"datatype\": \"%s\",\n", dimInfo.dataType.c_str());
        fprintf(fp, "            \"name\": \"%s\",\n", dimInfo.name.c_str());
        fprintf(fp, "            \"minimum\": %f,\n", dimInfo.minimum);
        fprintf(fp, "            \"mean\": %f,\n", dimInfo.mean);
        fprintf(fp, "            \"maximum\": %f\n", dimInfo.maximum);
        fprintf(fp, "        }%s\n", i++==numDims-1 ? "" : ",");
    }
    fprintf(fp, "    ]\n");
    fprintf(fp, "}\n");

    fclose(fp);
}

void RialtoFileWriter::writeTile(const std::string& tileSetName, PointView* view, uint32_t level, uint32_t col, uint32_t row, uint32_t mask)
{
    log()->get(LogLevel::Debug) << "RialtoFileWriter::writeTile()" << std::endl;

    const TileInfo tileInfo(view, level, col, row, mask);

    std::ostringstream os;

    os << m_directory;
    FileUtils::createDirectory(os.str());

    os << "/" << tileInfo.level;
    FileUtils::createDirectory(os.str());

    os << "/" << tileInfo.column;
    FileUtils::createDirectory(os.str());

    os << "/" << tileInfo.row << ".ria";
    FILE* fp = fopen(os.str().c_str(), "wb");

    // TODO: we don't write the number of points, but maybe we should
    
    if (!tileInfo.patch.isEmpty())
    {
        uint32_t bufLen = tileInfo.patch.size();
        const unsigned char* buf = tileInfo.patch.getPointer();
        fwrite(buf, bufLen, 1, fp);
    }

    uint8_t mask8 = tileInfo.mask;
    fwrite(&mask8, 1, 1, fp);

    fclose(fp);
}


std::string RialtoFileWriter::getName() const
{
    return s_info.name;
}


void RialtoFileWriter::processOptions(const Options& options)
{
    // we treat the target "filename" as the output directory,
    // so we'll use a differently named variable to make it clear
    m_directory = m_filename;

    m_tileSetName = options.getValueOrDefault<std::string>("tileSetName", "unnamed");
}


Options RialtoFileWriter::getDefaultOptions()
{
    Options options;
    return options;
}


void RialtoFileWriter::localStart()
{
    log()->get(LogLevel::Debug) << "RialtoFileWriter::localStart()" << std::endl;

    // pdal writers always clobber their output file, so we follow
    // the same convention here -- even though we're dealing with
    // an output "directory" instead of and output "file"
    if (FileUtils::directoryExists(m_filename))
    {
      FileUtils::deleteDirectory(m_filename);
    }

    if (!FileUtils::createDirectory(m_filename)) {
        throw pdal_error("RialtoFileWriter: Error creating directory");
    }
}


void RialtoFileWriter::localFinish()
{
    log()->get(LogLevel::Debug) << "RialtoFileWriter::localFinish()" << std::endl;
}

} // namespace rialto
} // namespace pdal
