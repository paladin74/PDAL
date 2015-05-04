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

#include <pdal/pdal_export.hpp>
#include <pdal/Writer.hpp>

#include <cstdint>
#include <string>


// A Rialto database contains two tables:
// 
// DataSets
//    id (PK)
//    name
//    bounds
//    levels
//    ...
//
// Tiles
//    id (PK)
//    dataset id (FK)
//    x, y, level


namespace rialtosupport
{
class SQLite;



class PDAL_DLL RialtoDb
{
public:
    enum Mode {
        Invalid,
        Create,
        Write,
        Read
    };

    // modes:
    //   Create: creates a new database and sets up the required tables
    //           it is an error if the db already exists
    //   Write: opens the database for writing
    //          it is an error if the db doesn't already exist
    //          it is also an error if the db doesn't have the required tables
    //   Read: opens an existing database
    //         it is an error if the db doesn't already exist
    RialtoDb(const std::string& path, Mode mode);
    ~RialtoDb();
    
    // construct a reader for the file, and send it to a db writer
    // (will also create the reproj and stats filters)
    // returns id of new data set
    uint32_t addDataSet(const std::string& filename);
    
    std::vector<uint32_t> getDataSetIds();
    std::string getDataSetInfo(uint32_t dataSetId);
    
    std::vector<uint32_t> getTileIds(/*bbox, levels*/);
    void getTileInfo(uint32_t tileId);
    
    int foo();
    
private:
    pdal::Writer* buildPipeline();
    void executePipeline(pdal::Writer*);
    
    void query();
    
    SQLite* m_sqlite;
    Mode m_mode;
    
    RialtoDb& operator=(const RialtoDb&); // not implemented
    RialtoDb(const RialtoDb&); // not implemented
};

} // namespace rialtosupport
