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

#include "GeoPackage.hpp"
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


class PDAL_DLL GeoPackageManager : public GeoPackage
{
public:
    // pass it the filename of the sqlite db
    GeoPackageManager(const std::string& connection, LogPtr log);

    virtual ~GeoPackageManager();

    virtual void open();
    virtual void close();

    virtual void childDumpStats() const;

private:
    void createGpkgId();
    void createTableGpkgSpatialRefSys();
    void createTableGpkgContents();
    void createTableGpkgPctileMatrixSet();
    void createTableGpkgPctileMatrix();
    void createTableGpkgMetadata();
    void createTableGpkgMetadataReference();
    void createTableGpkgExtensions();
    void createTablePctilesDimensionSet();

    Event e_creationOpen;
    Event e_creationClose;

    GeoPackageManager& operator=(const GeoPackageManager&); // not implemented
    GeoPackageManager(const GeoPackageManager&); // not implemented
};


} // namespace rialto

} // namespace pdal
