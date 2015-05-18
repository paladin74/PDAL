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
    inputFile(NULL), outputFile(NULL),
    inputType(TypeInvalid), outputType(TypeInvalid),
    maxLevel(20)
{   
    // TODO: for now, we only query at maxLevel
    qMinX=-189.9, qMinY=-89.9, qMaxX=189.9, qMaxY=89.9;
    haveQuery = false;

    // random defaults
    rMinX=-189.9, rMinY=-89.9, rMaxX=189.9, rMaxY=89.9;
    rCount = 1000;
    haveRandom = false;
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
            inputFile = argv[++i];
        }
        else if (streq(argv[i], "-o"))
        {
            outputFile = argv[++i];
        }
        else if (streq(argv[i], "-q"))
        {
            qMinX = atof(argv[++i]);
            qMinY = atof(argv[++i]);
            qMaxX = atof(argv[++i]);
            qMaxY = atof(argv[++i]);
            qBounds = BOX3D(qMinX, qMinY, qMaxX, qMaxY, -999999.999, 999999.999);
            haveQuery = true;
        }
        else if (streq(argv[i], "-r"))
        {
            rMinX = atof(argv[++i]);
            rMinY = atof(argv[++i]);
            rMaxX = atof(argv[++i]);
            rMaxY = atof(argv[++i]);
            rCount = atoi(argv[i++]);
            rBounds = BOX3D(rMinX, rMinY, rMaxX, rMaxY, -999999.999, 999999.999);
            haveRandom = true;
        }
        else if (streq(argv[i], "-v"))
        {
            haveVerify = true;
        }
        else
        {
            error("unrecognized option", argv[i]);
        }
        
        ++i;
    }
    
    if (!inputFile) error("input file not specified");
    if (!outputFile) error("output file not specified");
    
    inputType = inferType(inputFile);
    outputType = inferType(outputFile);
    
    switch (inputType)
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

    switch (outputType)
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
    
    if (haveRandom && inputType != TypeRandom)
    {
        error("random mode (-r) requires random input (-i random)");
    }
    
    if (haveQuery && inputType != TypeSqlite)
    {
        error("query mode (-q) requires sqlite input (-i NAME.sqlite)");
    }
}


Reader* Tool::createReader()
{
    return createReader(inputFile, inputType, rBounds, rCount);
}


Writer* Tool::createWriter()
{
    return createWriter(outputFile, outputType);
}


Reader* Tool::createReader(const char* name, FileType type, const BOX3D& rBounds, uint32_t rCount)
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
            reader = new RialtoDbReader();
            break;
        default:
            assert(0);
            break;
    }
    
    reader->setOptions(opts);
    return reader;
}


Writer* Tool::createWriter(const char* name, FileType type)
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
            writer = new RialtoDbWriter();
            break;
        case TypeTiles:
            opts.add("filename", name);
            writer = new RialtoFileWriter();
            break;
        default:
            assert(0);
            break;
    }
    
    writer->setOptions(opts);
    return writer;
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

        if (xA != xE) error("verify failed", "x");
        if (yA != yE) error("verify failed", "y");
        if (zA != zE) error("verify failed", "z");
    }
}


void Tool::verify()
{
    if (!haveVerify) return;
    
    BOX3D unusedBox;
    uint32_t unusedCount;
    
    Reader* readerExpected = createReader(inputFile, inputType, rBounds, rCount);
    Reader* readerActual = createReader(outputFile, outputType, unusedBox, unusedCount);

    PointViewSet viewsActual;
    PointViewPtr viewActual;
    PointViewSet viewsExpected;
    PointViewPtr viewExpected;

    pdal::PointTable tableActual;
    readerActual->prepare(tableActual);
    viewsActual = readerActual->execute(tableActual);

    pdal::PointTable tableExpected;
    readerExpected->prepare(tableExpected);
    viewsExpected = readerExpected->execute(tableExpected);

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
}
