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

static PluginInfo const s_info = PluginInfo(
    "writers.rialto",
    "Rialto Writer",
    "http://pdal.io/stages/writers.rialto.html" );

CREATE_STATIC_PLUGIN(1, 0, RialtoWriter, Writer, s_info)

namespace
{
    static uint32_t getMetadataU32(const MetadataNode& parent, const std::string& name)
    {
        const MetadataNode node = parent.findChild(name);
        if (!node.valid()) {
            throw pdal_error("RialtoWriter: required metadata item not found");
        }
        uint32_t v = boost::lexical_cast<uint32_t>(node.value());
        return v;
    }


    static double getMetadataF64(const MetadataNode& parent, const std::string& name)
    {
        const MetadataNode node = parent.findChild(name);
        if (!node.valid()) {
            throw pdal_error("RialtoWriter: required metadata item not found");
        }
        double v = boost::lexical_cast<double>(node.value());
        return v;
    }

    static void writeHeader(
            const std::string& directory,
            PointLayoutPtr layout,
            int32_t numCols,
            int32_t numRows,
            double minx,
            double miny,
            double maxx,
            double maxy)
    {
        const std::string filename(directory + "/header.json");
        FILE* fp = fopen(filename.c_str(), "wt");

        fprintf(fp, "{\n");
        fprintf(fp, "    \"version\": 4,\n");

        fprintf(fp, "    \"bbox\": [%lf, %lf, %lf, %lf],\n",
                minx, miny, maxx, maxy);

        fprintf(fp, "    \"numCols\": %d,\n", numCols);
        fprintf(fp, "    \"numRows\": %d,\n", numRows);

        fprintf(fp, "    \"dimensions\": [\n");

        const size_t numDims = layout->dims().size();
        size_t i = 0;
        for (const auto& dim : layout->dims())
        {
            const Dimension::Type::Enum dataType = layout->dimType(dim);
            std::string dataTypeName = Dimension::interpretationName(dataType);
            std::string name = Dimension::name(dim);

            // TODO: these will be optional
            double mind, meand, maxd;
//            const stats::Summary& d_stats = stats->getStats(dim);
//            mind = d_stats.minimum();
//            meand = d_stats.average();
//            maxd = d_stats.maximum();

            fprintf(fp, "        {\n");
            fprintf(fp, "            \"datatype\": \"%s\",\n", dataTypeName.c_str());
            fprintf(fp, "            \"name\": \"%s\",\n", name.c_str());
//            fprintf(fp, "            \"min\": %f,\n", mind);
//            fprintf(fp, "            \"mean\": %f,\n", meand);
//            fprintf(fp, "            \"max\": %f\n", maxd);
            fprintf(fp, "        }%s\n", i++==numDims-1 ? "" : ",");
        }
        fprintf(fp, "    ]\n");
        fprintf(fp, "}\n");

        fclose(fp);
    }


    static void fillBuffer(const PointView* view, const PointId& idx, char* buf)
    {
        char* p = buf;
        char* q = p;

        for (const auto& dim : view->dims())
        {
            view->getRawField(dim, idx, q);
            q += view->dimSize(dim);
        }
    }


    static void writeDataForOneTile(FILE* fp, PointView* view)
    {
        const size_t len = view->pointSize();
        char* buf = new char[len];

        for (size_t i=0; i<view->size(); ++i)
        {
            const PointId idx = i;
            fillBuffer(view, idx, buf);
            fwrite(buf, len, 1, fp);
        }

        delete[] buf;
    }


    static void writeOneTile(MetadataNode& tileNode, PointView* view, const std::string& directory)
    {
        const uint32_t level = getMetadataU32(tileNode, "level");
        const uint32_t tileX = getMetadataU32(tileNode, "tileX");
        const uint32_t tileY = getMetadataU32(tileNode, "tileY");
        const uint32_t mask = getMetadataU32(tileNode, "mask");

        std::ostringstream os;

        os << directory;
        FileUtils::createDirectory(os.str());

        os << "/" << level;
        FileUtils::createDirectory(os.str());

        os << "/" << tileX;
        FileUtils::createDirectory(os.str());

        os << "/" << tileY << ".ria";
        FILE* fp = fopen(os.str().c_str(), "wb");

        printf("tile %d/%d/%d  m%d\n", level, tileX, tileY, mask);
        if (view)
        {
          writeDataForOneTile(fp, view);
        }

        uint8_t mask8 = mask;
        fwrite(&mask8, 1, 1, fp);

        fclose(fp);
    }

} // anonymous namespace


std::string RialtoWriter::getName() const
{
    return s_info.name;
}


void RialtoWriter::processOptions(const Options& options)
{
    // we treat the target "filename" as the output directory,
    // so we'll use a differently named variable to make it clear
    m_directory = m_filename;
}


Options RialtoWriter::getDefaultOptions()
{
    Options options;
    return options;
}


void RialtoWriter::ready(PointTableRef table)
{
    printf("{RialtoWriter::ready}\n");

    m_table = &table;

    // pdal writers always clobber their output file, so we follow
    // the same convention here -- even though we're dealing with
    // an output "directory" instead of and output "file"
    if (FileUtils::directoryExists(m_filename))
    {
      FileUtils::deleteDirectory(m_filename);
    }

    if (!FileUtils::createDirectory(m_filename)) {
        throw pdal_error("RialtoWriter: Error creating directory");
    }

    MetadataNode tileSetNode = m_table->metadata().findChild("tileSet");
    if (!tileSetNode.valid()) {
        throw pdal_error("RialtoWriter: required TileFilter metadata \"tileSet\" not found (1)");
    }

    const uint32_t numTilesX = getMetadataU32(tileSetNode, "numCols");
    const uint32_t numTilesY = getMetadataU32(tileSetNode, "numRows");

    const double minx = getMetadataF64(tileSetNode, "minX");
    const double miny = getMetadataF64(tileSetNode, "minY");
    const double maxx = getMetadataF64(tileSetNode, "maxX");
    const double maxy = getMetadataF64(tileSetNode, "maxY");

    // TODO: hard-coded for now
    assert(minx==-180.0 && miny==-90.0 && maxx==180.0 && maxy==90.0);

    writeHeader(m_directory, m_table->layout(),
                numTilesX, numTilesY,
                minx, miny, maxx, maxy);

    // the metadata nodea are listed by tile id, not by point view id,
    // so we need make to make a map from point view id to metadata node
    const MetadataNode tilesNode = tileSetNode.findChild("tiles");
    if (!tilesNode.valid()) {
        throw pdal_error("RialtoWriter: required TileFilter metadata \"tiles\" not found");
    }
    const MetadataNodeList tileNodes = tilesNode.children();
    for (auto node: tileNodes)
    {
        MetadataNode n = node.findChild("pointView");
        if (n.valid()) {
          const uint32_t viewId = boost::lexical_cast<uint32_t>(n.value());
          m_dataMap[viewId] = node;
        }
    }
}


// write out the tile with this pointview
void RialtoWriter::write(const PointViewPtr view)
{
    printf("{RialtoWriter::write}\n");

    MetadataNode tileNode = m_dataMap[view->id()];
    assert(tileNode.valid());

    PointView* viewptr = view.get();
    writeOneTile(tileNode, viewptr, m_directory);
}


// write out all the remaining tiles: those without point views
void RialtoWriter::done(PointTableRef table)
{
  const MetadataNode tileSetNode = m_table->metadata().findChild("tileSet");
  const MetadataNode tilesNode = tileSetNode.findChild("tiles");
  const MetadataNodeList tileNodes = tilesNode.children();

  for (auto iter = tileNodes.begin(); iter != tileNodes.end(); ++iter)
  {
      MetadataNode tileNode = *iter;
      const MetadataNode nodeP = tileNode.findChild("pointView");
      if (!nodeP.valid()) {
        writeOneTile(tileNode, NULL, m_directory);
      }
  }

}


} // namespace pdal
