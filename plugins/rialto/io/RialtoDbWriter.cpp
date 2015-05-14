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



uint32_t RialtoDbWriter::writeHeader(const std::string& tileSetName,
                                 MetadataNode tileSetNode,
                                 PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::writeHeader()" << std::endl;

    RialtoDb::TileSetInfo tileSetInfo;
    serializeToTileSetInfo(tileSetName, tileSetNode, layout, tileSetInfo);

    const uint32_t tileSetId = m_rialtoDb->writeTileSet(tileSetInfo);

    return tileSetId;
}


void RialtoDbWriter::writeTile(uint32_t tileSetId, PointView* view, uint32_t level, uint32_t col, uint32_t row, uint32_t mask)
{
    log()->get(LogLevel::Debug1) << "RialtoDbWriter::writeTile()" << std::endl;

    RialtoDb::TileInfo tileInfo;
    serializeToTileInfo(tileSetId, view, tileInfo, level, col, row, mask);

    if (tileInfo.patch.buf.size())
    {
        uint32_t id = m_rialtoDb->writeTile(tileInfo);
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
    
    m_tileSetName = options.getValueOrDefault<std::string>("tileSetName", "unnamed");
}


Options RialtoDbWriter::getDefaultOptions()
{
    Options options;
    return options;
}


void RialtoDbWriter::localStart()
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::localStart()" << std::endl;

    // pdal writers always clobber their output file, so we follow
    // the same convention here -- even though we're dealing with
    // an output "database" instead of an output "file"

    FileUtils::deleteFile(m_connection);

    m_rialtoDb = new RialtoDb(m_connection, log());
    m_rialtoDb->create();
}


void RialtoDbWriter::localFinish()
{
    log()->get(LogLevel::Debug) << "RialtoDbWriter::localFinish()" << std::endl;

    m_rialtoDb->close();
    delete m_rialtoDb;
    m_rialtoDb = NULL;
}


} // namespace pdal
