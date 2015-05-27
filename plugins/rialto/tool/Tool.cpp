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

#include <pdal/pdal.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/BufferReader.hpp>
#include <../io/null/NullWriter.hpp>
#include <../io/faux/FauxReader.hpp>
#include <../io/las/LasReader.hpp>
#include <../io/las/LasWriter.hpp>
#include <../filters/tiler/TilerFilter.hpp>
#include <../filters/stats/StatsFilter.hpp>
#include <../filters/reprojection/ReprojectionFilter.hpp>
#include <../plugins/rialto/io/RialtoDbReader.hpp>
#include <../plugins/rialto/io/RialtoDbWriter.hpp>
#include <../plugins/rialto/io/RialtoFileWriter.hpp>

#include "Tool.hpp"

using namespace pdal;


static void error(const char* p, const char* q=NULL)
{
    fprintf(stderr, "ERROR: %s", p);
    if (q)
    {
        fprintf(stderr, ": %s", q);
    }
    fprintf(stderr, "\n");
    exit(1);
}


static bool streq(const char* p, const char* q)
{
    return (strcmp(p,q)==0);
}


Tool::Tool() :
    m_inputName(NULL),
    m_outputName(NULL),
    m_inputType(TypeInvalid),
    m_outputType(TypeInvalid),
    m_maxLevel(15),
    m_rBounds(BOX3D(-189.9, -89.9, 189.9, 89.9, -999999.0, 999999.0)),
    m_qBounds(BOX3D(-189.9, -89.9, 189.9, 89.9, -999999.0, 999999.0)),
    m_haveQuery(false),
    m_haveRandom(false),
    m_rCount(1000)
{
    Utils::random_seed(17);
}


Tool::~Tool()
{}


Tool::FileType Tool::inferType(const char* p)
{
    if (streq(p, "null")) return TypeNull;
    if (streq(p, "random")) return TypeRandom;

    const char* ext = strrchr(p, '.');
    if (streq(ext, ".las")) return TypeLas;
    if (streq(ext, ".laz")) return TypeLaz;
    if (streq(ext, ".json")) return TypeTiles;
    if (streq(ext, ".sqlite")) return TypeSqlite;

    return TypeInvalid;
}


void Tool::processOptions(int argc, char* argv[])
{
    int i = 1;

    while (i < argc)
    {
        if (streq(argv[i], "-i"))
        {
            m_inputName = argv[++i];
        }
        else if (streq(argv[i], "-o"))
        {
            m_outputName = argv[++i];
        }
        else if (streq(argv[i], "-q"))
        {
            m_qBounds.minx = atof(argv[++i]);
            m_qBounds.miny = atof(argv[++i]);
            m_qBounds.maxx = atof(argv[++i]);
            m_qBounds.maxy = atof(argv[++i]);
            m_haveQuery = true;
        }
        else if (streq(argv[i], "-r"))
        {
            m_rBounds.minx = atof(argv[++i]);
            m_rBounds.miny = atof(argv[++i]);
            m_rBounds.maxx = atof(argv[++i]);
            m_rBounds.maxy = atof(argv[++i]);
            m_rCount = atoi(argv[i++]);
            m_haveRandom = true;
        }
        else if (streq(argv[i], "-v"))
        {
            m_haveVerify = true;
        }
        else
        {
            error("unrecognized option", argv[i]);
        }

        ++i;
    }

    if (!m_inputName) error("input file not specified");
    if (!m_outputName) error("output file not specified");

    m_inputType = inferType(m_inputName);
    m_outputType = inferType(m_outputName);

    switch (m_inputType)
    {
        case TypeRandom:
        case TypeLas:
        case TypeLaz:
        case TypeSqlite:
            // ok
            break;
        case TypeNull:
        case TypeTiles:
        case TypeInvalid:
            error("input file type not supported");
        default:
            assert(0);
    }

    switch (m_outputType)
    {
        case TypeNull:
        case TypeLas:
        case TypeLaz:
        case TypeSqlite:
        case TypeTiles:
            // ok
            break;
        case TypeRandom:
        case TypeInvalid:
            error("output file type not supported");
        default:
            assert(0);
    }

    if (m_haveRandom && m_inputType != TypeRandom)
    {
        error("random mode (-r) requires random input (-i random)");
    }

    if (m_haveQuery && m_inputType != TypeSqlite)
    {
        error("query mode (-q) requires sqlite input (-i NAME.sqlite)");
    }
}


Stage* Tool::createReader()
{
    return createReader(m_inputName, m_inputType, m_rBounds, m_rCount);
}


Stage* Tool::createWriter()
{
    return createWriter(m_outputName, m_outputType);
}


Stage* Tool::createReader(const char* name, FileType type, const BOX3D& rBounds, uint32_t rCount)
{
    Options opts;
    Reader* reader = NULL;

    switch (type)
    {
        case TypeRandom:
            opts.add("bounds", rBounds);
            opts.add("count", rCount);
            opts.add("mode", "random");
            reader = new FauxReader();
            break;
        case TypeLas:
            opts.add("filename", name);
            reader = new LasReader();
            break;
        case TypeLaz:
            opts.add("filename", name);
            reader = new LasReader();
            break;
        case TypeSqlite:
            opts.add("filename", name);
            reader = new rialto::RialtoDbReader();
            break;
        default:
            assert(0);
            break;
    }

    reader->setOptions(opts);
    return reader;
}


Stage* Tool::createWriter(const char* name, FileType type)
{
    FileUtils::deleteFile(name);

    Options opts;
    Writer* writer = NULL;

    switch (type)
    {
        case TypeNull:
            writer = new NullWriter();
            break;
        case TypeLas:
            opts.add("filename", name);
            writer = new LasWriter();
            opts.add("compressed", false);
            break;
        case TypeLaz:
            opts.add("filename", name);
            writer = new LasWriter();
            opts.add("compressed", true);
            break;
        case TypeSqlite:
            opts.add("filename", name);
            writer = new rialto::RialtoDbWriter();
            break;
        case TypeTiles:
            opts.add("filename", name);
            writer = new rialto::RialtoFileWriter();
            break;
        default:
            assert(0);
            break;
    }

    writer->setOptions(opts);
    return writer;
}


Stage* Tool::createReprojectionFilter()
{
    Options reprojOptions;
    ReprojectionFilter* reproj = new ReprojectionFilter();
    reprojOptions.add("out_srs", "EPSG:4326");
    reproj->setOptions(reprojOptions);
    return reproj;
}


Stage* Tool::createStatsFilter()
{
    Options statsOptions;
    StatsFilter* stats = new StatsFilter();
    stats->setOptions(statsOptions);
    return stats;
}


Stage* Tool::createTilerFilter()
{
    Options tilerOptions;
    tilerOptions.add("maxLevel", m_maxLevel);
    tilerOptions.add("numCols", 2);
    tilerOptions.add("numRows", 1);
    tilerOptions.add("minx", -180.0);
    tilerOptions.add("miny", -90.0);
    tilerOptions.add("maxx", 180.0);
    tilerOptions.add("maxy", 90.0);
    TilerFilter* tiler = new TilerFilter();
    tiler->setOptions(tilerOptions);
    return tiler;
}


static void verifyPoints(PointViewPtr viewA, PointViewPtr viewE)
{
    // TODO: we only test X, Y, Z

    for (uint32_t i=0; i<viewA->size(); i++)
    {
        const double xA = viewA->getFieldAs<double>(Dimension::Id::X, i);
        const double yA = viewA->getFieldAs<double>(Dimension::Id::Y, i);
        const double zA = viewA->getFieldAs<double>(Dimension::Id::Z, i);

        const double xE = viewE->getFieldAs<double>(Dimension::Id::X, i);
        const double yE = viewE->getFieldAs<double>(Dimension::Id::Y, i);
        const double zE = viewE->getFieldAs<double>(Dimension::Id::Z, i);

        if (xA != xE || yA != yE || zA != zE)
        {
          char buf[1024];
          sprintf(buf, "%i:\n\txA=%f\txE=%f\n\tyA=%f\tyE=%f\n\tzA=%f\tzE=%f\n",
              i, xA, xE, yA, yE, zA, zE);
          error("verify failed", buf);
        }
    }
}


void Tool::verify()
{
    if (!m_haveVerify) return;

    BOX3D unusedBox;
    uint32_t unusedCount;

    Stage* readerExpected1 = createReader(m_inputName, m_inputType, m_rBounds, m_rCount);
    Stage* readerExpected2 = createReprojectionFilter();
    readerExpected2->setInput(*readerExpected1);
    Stage* readerActual = createReader(m_outputName, m_outputType, unusedBox, unusedCount);

    PointViewSet viewsActual;
    PointViewPtr viewActual;
    PointViewSet viewsExpected;
    PointViewPtr viewExpected;

    pdal::PointTable tableActual;
    readerActual->prepare(tableActual);
    viewsActual = readerActual->execute(tableActual);

    pdal::PointTable tableExpected;
    readerExpected2->prepare(tableExpected);
    viewsExpected = readerExpected2->execute(tableExpected);

    if (viewsActual.size() != viewsExpected.size() || viewsActual.size() != 1)
    {
        error("verify failed", "unequal view set sizes");
    }

    viewActual = *(viewsActual.begin());
    viewExpected = *(viewsExpected.begin());

    // if the sizes are equal, we'll assume a one-way compare is okay
    if (viewActual->size() != viewExpected->size())
    {
        error("verify failed", "unequal view sizes");
    }

    verifyPoints(viewActual, viewExpected);
    printf("verify passed\n");
}
