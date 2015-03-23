/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include "TIndexKernel.hpp"

#include <memory>

#include <pdal/PDALUtils.hpp>
#include <ogr_api.h>

#include <boost/program_options.hpp>



namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.tindex",
    "TIndex Kernel",
    "http://pdal.io/kernels/kernels.tindex.html" );

CREATE_STATIC_PLUGIN(1, 0, TIndexKernel, Kernel, s_info)

std::string TIndexKernel::getName() const { return s_info.name; }

TIndexKernel::TIndexKernel()
    : Kernel()
    , m_outputFilename("")

{}


void TIndexKernel::validateSwitches()
{
    if (!m_outputFilename.size())
        throw app_runtime_error("No source file given!");
}


void TIndexKernel::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("directory", po::value<std::string>(&m_indexDirectory), "Directory to index")
        ("filename", po::value<std::string>(&m_outputFilename), "OGR-writeable tile index output")
    ;

    addSwitchSet(file_options);
    po::options_description* processing_options =
        new po::options_description("processing options");

    processing_options->add_options();

    addSwitchSet(processing_options);

    addPositionalSwitch("filename", 1);
}


OGRGeometryH fetchGeometry(std::string const& filename)
{
    Options readerOptions;

    readerOptions.add<std::string>("filename", filename);
//     setCommonOptions(readerOptions);
    std::unique_ptr<PipelineManager> m_manager;



    std::unique_ptr<PipelineManager> manager = std::unique_ptr<PipelineManager>(
                                        KernelSupport::makePipeline(filename));

    Stage* stage = manager->getStage();
    stage->setOptions(readerOptions);

    Stage* hexbinStage = &(m_manager->addFilter("filters.hexbin"));
    Options hexOptions;
    hexbinStage->setOptions(hexOptions);
    hexbinStage->setInput(*stage);

    stage = hexbinStage;

    PointTable tbl;
    stage->prepare(tbl);
    stage->execute(tbl);

    OGRGeometryH output(0);
    return output;



}


int TIndexKernel::execute()
{


    return 0;
}

} // namespace pdal

