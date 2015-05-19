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
#include <pdal/Reader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Writer.hpp>

using namespace pdal;


class Tool
{
public:
    enum FileType
    {
        TypeInvalid,
        TypeLas,        // rw
        TypeLaz,        // rw
        TypeSqlite,     // rw
        TypeTiles,      // w
        TypeRandom,     // r
        TypeNull        // w
    };

    Tool();
    ~Tool();

    void processOptions(int argc, char* argv[]);

    Stage* createReader();
    Stage* createWriter();
    Stage* createReprojectionFilter();
    Stage* createStatsFilter();
    Stage* createTilerFilter();

    void verify();

private:
    static FileType inferType(const char* p);
    static Stage* createReader(const char* name, FileType type, const BOX3D& rBounds, uint32_t rCount);
    static Stage* createWriter(const char* name, FileType type);

    const char* m_inputName;
    const char* m_outputName;

    FileType m_inputType;
    FileType m_outputType;

    bool m_haveVerify;
    BOX3D m_rBounds;
    BOX3D m_qBounds;
    bool m_haveQuery;
    bool m_haveRandom;

    uint32_t m_maxLevel; // TODO

    // TODO: for now, we only query at maxLevel

    uint32_t m_rCount;
};
