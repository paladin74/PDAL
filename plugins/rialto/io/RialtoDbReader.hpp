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

#include <pdal/Reader.hpp>


namespace pdal
{
namespace rialto
{


class TileTableInfo;
class RialtoDb;

    
class PDAL_DLL RialtoDbReader : public Reader
{
public:
    RialtoDbReader();
    ~RialtoDbReader();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

    const TileTableInfo& getTileTableInfo() const { return *m_tileTableInfo; }
    
protected:
  virtual void processOptions(const Options& options);
  virtual void initialize();
  virtual void addDimensions(PointLayoutPtr layout);
  virtual void ready(PointTableRef table);
  point_count_t read(PointViewPtr view, point_count_t count);

private:
  RialtoDb* m_db;
  std::string m_tileTableName;
  uint32_t m_level;
  std::unique_ptr<TileTableInfo> m_tileTableInfo;
  BOX3D m_query;
  
  RialtoDbReader& operator=(const RialtoDbReader&); // not implemented
  RialtoDbReader(const RialtoDbReader&); // not implemented
};

} // rialto namespace
} // pdal namespace
