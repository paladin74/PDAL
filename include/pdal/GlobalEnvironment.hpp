/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/pdal_internal.hpp>
#include <pdal/Log.hpp>

#ifdef PDAL_HAVE_PYTHON
#include <pdal/plang/PythonEnvironment.hpp>
#endif

#include <mutex>
#include <memory>

namespace pdal
{
namespace gdal
{
class ErrorHandler;
}

class PDAL_DLL GlobalEnvironment
{
public:
    static GlobalEnvironment& get();
    static void startup();
    static void shutdown();

#ifdef PDAL_HAVE_PYTHON
    void createPythonEnvironment();
    plang::PythonEnvironment& getPythonEnvironment();
#endif

    void initializeGDAL(LogPtr log, bool bGDALDebugOutput=false);

private:
    GlobalEnvironment();
    ~GlobalEnvironment();

    std::unique_ptr<gdal::ErrorHandler> m_gdalDebug;
#ifdef PDAL_HAVE_PYTHON
    std::unique_ptr<plang::PythonEnvironment> m_pythonEnvironment;
#endif

    GlobalEnvironment(const GlobalEnvironment&); // nope
    GlobalEnvironment& operator=(const GlobalEnvironment&); // nope
};

} // namespace pdal

