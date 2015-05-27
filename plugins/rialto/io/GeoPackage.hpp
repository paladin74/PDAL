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

#include <pdal/pdal.hpp>

#include "Event.hpp"


namespace pdal
{
    class Log;
    class SQLite;

namespace rialto
{
    
class GpkgMatrixSet;
class GpkgTile;
class GpkgDimension;

class PDAL_DLL GeoPackage
{
public:
    // pass it the filename of the sqlite db
    GeoPackage(const std::string& connection, LogPtr log);

    virtual ~GeoPackage();

    void beginTransaction();
    void commitTransaction();

    virtual void open() = 0;
    virtual void close() = 0;

    
    // get info about a specific tile matrix set (including its dimensions)
    void readMatrixSet(std::string const& name, GpkgMatrixSet& info) const;

    // get list of names all the matrix sets ("files") in the db
    void readMatrixSetNames(std::vector<std::string>&) const;

    // get list of all the matrix sets ("files") in the db
    void readMatrixSets(std::vector<GpkgMatrixSet>&) const;

    virtual void dumpStats() const {};

protected:
    void internalOpen(bool writable);
    void internalClose();
    
    // get info about one of the dimensions of a tile set
    void readDimensions(std::string const& name, std::vector<GpkgDimension>&) const;

    uint32_t querySrsId(const std::string& wkt) const;
    std::string querySrsWkt(uint32_t srs_id) const;

    void verifyTableExists(std::string const& name) const;

    LogPtr log() const { return m_log; }

    std::unique_ptr<SQLite> m_sqlite;

private:
    std::string m_connection;
    LogPtr m_log;
    
    GeoPackage& operator=(const GeoPackage&); // not implemented
    GeoPackage(const GeoPackage&); // not implemented
};


} // namespace rialto

} // namespace pdal
