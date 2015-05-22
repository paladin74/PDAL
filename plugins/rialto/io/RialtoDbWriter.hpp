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

#include <pdal/Writer.hpp>
#include "RialtoSupport.hpp"

extern "C" int32_t RialtoDbWriter_ExitFunc();
extern "C" PF_ExitFunc RialtoDbWriter_InitPlugin();


namespace pdal
{
namespace rialto
{
    class RialtoDb;
    

class DbWriterAssister: public WriterAssister
{
public:
    RialtoDb* m_rialtoDb;
    
private:
    virtual void writeHeader(const std::string& tileSetName,
                             MetadataNode tileSetNode,
                             PointLayoutPtr layout,
                             const std::string& datetime);
    virtual void writeTile(const std::string& tileSetName, PointView*,
                           uint32_t level, uint32_t col, uint32_t row, uint32_t mask);
};


class PDAL_DLL RialtoDbWriter : public Writer
{
public:
    RialtoDbWriter()
    {}

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

    void ready(PointTableRef table);
    void write(const PointViewPtr viewPtr);
    void done(PointTableRef table);

private:
    void processOptions(const Options& options);

    DbWriterAssister m_assister;

    std::string m_connection;
    RialtoDb* m_rialtoDb;

    RialtoDbWriter& operator=(const RialtoDbWriter&); // not implemented
    RialtoDbWriter(const RialtoDbWriter&); // not implemented
};

} // namespace rialto
} // namespace pdal
