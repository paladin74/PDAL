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

#include "GeoPackage.hpp"
#include "GeoPackageCommon.hpp"

#include <pdal/../../plugins/sqlite/io/SQLiteCommon.hpp> // TODO: fix path
#include <pdal/../../filters/tiler/TilerCommon.hpp> // TODO: fix path


namespace pdal
{

namespace rialto
{


GeoPackage::GeoPackage(const std::string& connection, LogPtr log) :
    m_connection(connection),
    m_log(log)
{
    //m_log->setLevel(LogLevel::Debug);
}


GeoPackage::~GeoPackage()
{
    assert(!m_sqlite);

    log()->get(LogLevel::Debug) << "~RialtoDB" << std::endl;
}


void GeoPackage::internalOpen(bool writable)
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    log()->get(LogLevel::Debug) << "RialtoDB::create()" << std::endl;

    if (!writable && !FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: does not exist");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(writable);
}


void GeoPackage::internalClose()
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    m_sqlite.reset();

    dumpStats();
}


uint32_t GeoPackage::querySrsId(const std::string& wkt) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    // TODO: how better to look up SRS than text match of WKT?
    
    std::ostringstream oss;
    oss << "SELECT srs_id FROM gpkg_spatial_ref_sys"
        << " WHERE definition='" << wkt << "'";

    m_sqlite->query(oss.str());

    const row* r = m_sqlite->get();
    if (!r) return 0;

    // take the first one, ignore any others
    const uint32_t srs_id = boost::lexical_cast<uint32_t>(r->at(0).data);

    return srs_id;
}


void GeoPackage::verifyTableExists(std::string const& name) const
{
    if (!m_sqlite->doesTableExist(name))
    {
        throw pdal_error("RialtoDb: required table '" + name + "' not found");
    }
}


void GeoPackage::beginTransaction()
{
    m_sqlite->begin();
}


void GeoPackage::commitTransaction()
{
    m_sqlite->commit();    
}


} // namespace rialto
} // namespace pdal
