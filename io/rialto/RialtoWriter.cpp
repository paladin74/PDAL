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
    static void writeHeader(
            std::string dir,
            PointTableRef table,
            int32_t xt,
            int32_t yt,
            double minx,
            double miny,
            double maxx,
            double maxy)
    {
        std::string filename(dir + "/header.json");
        FILE* fp = fopen(filename.c_str(), "wt");

        fprintf(fp, "{\n");
        fprintf(fp, "    \"version\": 3,\n");

        fprintf(fp, "    \"tilebbox\": [%f, %f, %f, %f],\n",
                minx, miny, maxx, maxy);

        fprintf(fp, "    \"numTilesX\": %d,\n", xt);
        fprintf(fp, "    \"numTilesY\": %d,\n", yt);

        fprintf(fp, "    \"databbox\": [%f, %f, %f, %f],\n",
                minx, miny, maxx, maxy);

        const PointLayoutPtr layout(table.layout());
        const size_t numDims = layout->dims().size();
        fprintf(fp, "    \"dimensions\": [\n");

        size_t i = 0;
        for (const auto& dim : layout->dims())
        {
            const Dimension::Type::Enum dataType = layout->dimType(dim);
            std::string dataTypeName = Dimension::interpretationName(dataType);
            std::string name = Dimension::name(dim);

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
    
    
    char* getPointData(PointViewPtr view, PointId& idx)
    {
        char* p = new char[view->pointSize()];
        char* q = p;

        for (const auto& dim : view->dims())
        {
            view->getRawField(dim, idx, q);
            q += view->dimSize(dim);
        }

        return p;
    }


    static void writeDataForOneTile(FILE* fp, PointViewPtr view)
    {
        //const PointLayoutPtr layout(view.layout());

        for (size_t i=0; i<view->size(); ++i)
        {
            PointId idx = i;
            char* p = getPointData(view, idx);

            for (const auto& dim : view->dimTypes())
            {
                size_t size = Dimension::size(dim.m_type);

                fwrite(p, size, 1, fp);

                p += size;
            }
        }
    }


    static void writeOneTile(MetadataNode& tileNode, PointViewPtr view, const char* prefix)
    {
        const MetadataNode nodeL = tileNode.findChild("level");
        const MetadataNode nodeX = tileNode.findChild("tileX");
        const MetadataNode nodeY = tileNode.findChild("tileY");
        const MetadataNode nodeM = tileNode.findChild("mask");
        assert(nodeL.valid());
        assert(nodeX.valid());
        assert(nodeY.valid());
        assert(nodeM.valid());
        const uint32_t level = boost::lexical_cast<uint32_t>(nodeL.value());
        const uint32_t tileX = boost::lexical_cast<uint32_t>(nodeX.value());
        const uint32_t tileY = boost::lexical_cast<uint32_t>(nodeY.value());
        const uint8_t mask = boost::lexical_cast<uint32_t>(nodeM.value());

        char* filename = new char[strlen(prefix) + 1024];

        sprintf(filename, "%s", prefix);
        FileUtils::createDirectory(filename);

        sprintf(filename, "%s/%d", prefix, level);
        FileUtils::createDirectory(filename);

        sprintf(filename, "%s/%d/%d", prefix, level, tileX);
        FileUtils::createDirectory(filename);

        sprintf(filename, "%s/%d/%d/%d.ria", prefix, level, tileX, tileY);

        printf("FILENAME: %s\n", filename);
        
        FILE* fp = fopen(filename, "wb");

        writeDataForOneTile(fp, view);

        fwrite(&mask, 1, 1, fp);

        fclose(fp);

        delete[] filename;
    }

} // anonymous namespace

std::string RialtoWriter::getName() const
{
    return s_info.name;
}

void RialtoWriter::processOptions(const Options& options)
{
    m_overwrite = options.getValueOrDefault<bool>("overwrite", false);
}

Options RialtoWriter::getDefaultOptions()
{
    Options options;
    options.add("max_level", 16, "Max number of levels");
    options.add("overwrite", false, "Overwrite existing files?");
    return options;
}


uint32_t RialtoWriter::getMetadataU32(const MetadataNode& parent, const std::string& name) const {
    const MetadataNode node = parent.findChild(name);
    if (!node.valid()) {
        throw pdal_error("RialtoWriter: required metadata item not found");
    }
    uint32_t v = boost::lexical_cast<uint32_t>(node.value());
    return v;
}


double RialtoWriter::getMetadataF64(const MetadataNode& parent, const std::string& name) const {
    const MetadataNode node = parent.findChild(name);
    if (!node.valid()) {
        throw pdal_error("RialtoWriter: required metadata item not found");
    }
    double v = boost::lexical_cast<double>(node.value());
    return v;
}


void RialtoWriter::ready(PointTableRef table)
{
    printf("{RialtoWriter::ready}\n");

    m_table = &table;

    MetadataNode tileSetNode = m_table->metadata().findChild("tileSet");
    if (!tileSetNode.valid()) {
        throw pdal_error("RialtoWriter: required TileFilter metadata \"tileSet\" not found (1)");
    }


    if (FileUtils::directoryExists(m_filename))
    {
        if (!m_overwrite)
            throw pdal_error("RialtoWriter: Requested directory already exists. "\
                "Use writers.rialto.overwrite to delete the existing directory.\n");
        else
            FileUtils::deleteDirectory(m_filename);
    }

    if (!FileUtils::createDirectory(m_filename))
        throw pdal_error("RialtoWriter: Error creating directory.\n");
        
    uint32_t numTilesX = getMetadataU32(tileSetNode, "numCols");
    uint32_t numTilesY = getMetadataU32(tileSetNode, "numRows");

    const double minx = getMetadataF64(tileSetNode, "minX");
    const double miny = getMetadataF64(tileSetNode, "minY");
    const double maxx = getMetadataF64(tileSetNode, "maxX");
    const double maxy = getMetadataF64(tileSetNode, "maxY");

    // TODO: hard-coded for now
    assert(minx==-180.0 && miny==-90.0 && maxx==180.0 && maxy==90.0);

    writeHeader(
            m_filename,
            *m_table,
            numTilesX,
            numTilesY,
            minx, miny, maxx, maxy);
}


void RialtoWriter::write(const PointViewPtr view)
{
    printf("{RialtoWriter::write}\n");
    
    printf("view %d\n", view->id());

    const PointView& viewRef(*view.get());

    MetadataNode tileSetNode = m_table->metadata().findChild("tileSet");

    const MetadataNode tilesNode = tileSetNode.findChild("tiles");
    if (!tilesNode.valid()) {
        throw pdal_error("RialtoWriter: required TileFilter metadata \"tiles\" not found");
    }
    const MetadataNodeList tileNodes = tilesNode.children();

    const std::string idString = std::to_string(view->id());
    MetadataNode tileNode = tilesNode.findChild(idString);
    assert(tileNode.valid());

#if 0
    // dump tile info
        int32_t num_levels = m_maxLevel+1;
        std::vector<int32_t> numTilesPerLevel(num_levels, 0);
        std::vector<int64_t> numPointsPerLevel(num_levels, 0);

        m_roots[0]->collectStats(numTilesPerLevel, numPointsPerLevel);
        m_roots[1]->collectStats(numTilesPerLevel, numPointsPerLevel);
#endif

    writeOneTile(tileNode, view, m_filename.c_str());
}

void RialtoWriter::done(PointTableRef table)
{
}


} // namespace pdal
