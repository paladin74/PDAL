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
#include "../info/InfoKernel.hpp"

#ifndef WIN32
#include <glob.h>
#endif

#include <memory>
#include <vector>

#include <pdal/PDALUtils.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/util/FileUtils.hpp>

#include <ogr_api.h>
#include <ogr_srs_api.h>
#include <cpl_port.h>
#include <cpl_vsi.h>
#include <cpl_string.h>


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
    , m_layerName("")
    , m_driverName("ESRI Shapefile")
    , m_tileIndexColumnName("location")
    , m_srsColumnName("")
    , m_kernelFactory(0)
    , m_DS(0)
    , m_Layer(0)
    , m_fDefn(0)
    , m_bAbsolutePaths(false)
    , m_targetSRSString("")
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
        ("filename", po::value<std::string>(&m_outputFilename), "OGR-writeable tile index output")
        ("directory", po::value<std::string>(&m_indexDirectory), "Directory to index")
        ("lyr_name", po::value<std::string>(&m_layerName), "OGR layer name to write into datasource")
        ("tile_index", po::value<std::string>(&m_tileIndexColumnName), "Tile index column name")
        ("src_srs_name", po::value<std::string>(&m_srsColumnName), "SRS column name")
        ("driver_name,f", po::value<std::string>(&m_driverName)->default_value("ESRI Shapefile"), "OGR driver name to use ")
        ("write_absolute_paths,p", po::value<bool>(&m_bAbsolutePaths)->zero_tokens()->implicit_value(true), "Write absolute paths")
        ("t_srs", po::value<std::string>(&m_targetSRSString), "Target SRS of tile index")
    ;

    addSwitchSet(file_options);
    po::options_description* processing_options =
        new po::options_description("processing options");

    processing_options->add_options();

    addSwitchSet(processing_options);

    addPositionalSwitch("filename", 1);
    addPositionalSwitch("directory", 2);
}



inline std::vector<std::string> glob(const std::string& pat){
    using namespace std;
    vector<string> ret;
#ifndef WIN32
    glob_t glob_result;
    glob(pat.c_str(),GLOB_TILDE,NULL,&glob_result);
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        ret.push_back(string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
#endif
    return ret;
}


MetadataNode TIndexKernel::fetchInfo(std::string const& filename)
{
    // kf is void* because we don't have definition when 
    // we make TIndexKernel 
    KernelFactory* kf = static_cast<KernelFactory*>(m_kernelFactory);
    std::unique_ptr<Kernel> app = kf->createKernel("kernels.info");
    
    Kernel* k = static_cast<Kernel*>(app.get());
    // Evil, this is.
    InfoKernel* info = static_cast<InfoKernel*>(k);

    info->doShowAll(true);
    info->prepare(filename);

    MetadataNode metadata;
    try
    {
        metadata = info->dump(filename);
    } catch (...)
    {
        // empty metdata
    }

    // done with the kernel.
    app.release();

    return metadata;
}


void TIndexKernel::createDS(std::string const& filename)
{
    m_DS = OGROpen( filename.c_str(), TRUE, NULL );

    if (m_DS != NULL)
    {
        if( OGR_DS_GetLayerCount(m_DS) == 1 )
            m_Layer = OGR_DS_GetLayer(m_DS, 0);
        else
        {
            if( m_layerName.size() == 0 )
            {
                std::cerr << "--lyr_name must be specified because there is more than one layer in DS" << std::endl;
                exit( 1 );
            }
            m_Layer = OGR_DS_GetLayerByName(m_DS, m_layerName.c_str());
        }
    }
    else
    {
        OGRSFDriverH hDriver;

        std::clog << "Creating new index file ..." << std::endl;
        hDriver = OGRGetDriverByName( m_driverName.c_str() );
        if( hDriver == NULL )
        {
            std::cerr << "'" << m_driverName << "' driver is not available" << std::endl;
            exit( 1 );
        }


        std::string dsname = pdal::FileUtils::toAbsolutePath(filename);
        std::cout << "creating datasource with filename : '" << dsname<< "'" << std::endl;

        m_DS = OGR_Dr_CreateDataSource( hDriver, dsname.c_str(), NULL );
    }

}

void TIndexKernel::createLayer(std::string const& filename, std::string const& srs_wkt)
{
    size_t nMaxFieldSize(254);

    if( m_DS != NULL && m_Layer == NULL )
    {
        if( m_layerName.c_str() == NULL )
        {
            VSIStatBuf sStat;
            if( EQUAL(m_driverName.c_str(), "ESRI Shapefile") ||
                VSIStat(filename.c_str(), &sStat) == 0 )
                m_layerName = CPLStrdup(CPLGetBasename(filename.c_str()));
            else
            {
                std::cerr << "--lyr_name must be specified because there is more than one layer in DS" << std::endl;
                exit( 1 );
            }
        }

        OGRSpatialReferenceH hSpatialRef = OSRNewSpatialReference(srs_wkt.c_str());
        m_Layer = OGR_DS_CreateLayer( m_DS, m_layerName.c_str(), hSpatialRef, wkbPolygon, NULL );
        if (hSpatialRef)
            OSRRelease(hSpatialRef);

        if (m_Layer)
        {
            OGRFieldDefnH hFieldDefn = OGR_Fld_Create( m_tileIndexColumnName.c_str(), OFTString );
            if( nMaxFieldSize )
                OGR_Fld_SetWidth( hFieldDefn, nMaxFieldSize);
            OGR_L_CreateField( m_Layer, hFieldDefn, TRUE );
            OGR_Fld_Destroy(hFieldDefn);
            if( m_srsColumnName.size() )
            {
                hFieldDefn = OGR_Fld_Create( m_srsColumnName.c_str(), OFTString );
                if( nMaxFieldSize )
                    OGR_Fld_SetWidth( hFieldDefn, nMaxFieldSize);
                OGR_L_CreateField( m_Layer, hFieldDefn, TRUE );
                OGR_Fld_Destroy(hFieldDefn);
            }
        }
    }

    if( m_DS == NULL || m_Layer == NULL )
    {
        std::cerr << "Unable to create layer '" << filename << "'" << std::endl;
        exit(2);
    }

    m_fDefn = OGR_L_GetLayerDefn(m_Layer);

    int ti_field = OGR_FD_GetFieldIndex( m_fDefn, m_tileIndexColumnName.c_str());
    if( ti_field < 0 )
    {
        std::cerr << "Unable to find field '" << m_tileIndexColumnName << "' in file '" << filename <<"'" << std::endl;
        exit(2);
    }
    
//     if( m_srsColumnName.size() )
//         i_SrcSRSName = OGR_FD_GetFieldIndex( hFDefn, pszSrcSRSName );

    /* Load in memory existing file names in SHP */
    int nExistingFiles = (int)OGR_L_GetFeatureCount(m_Layer, FALSE);
    if (nExistingFiles > 0)
    {
        OGRFeatureH hFeature;
        for(auto i=0;i<nExistingFiles;i++)
        {
            hFeature = OGR_L_GetNextFeature(m_Layer);
            m_files.push_back(OGR_F_GetFieldAsString( hFeature, ti_field));
            OGR_F_Destroy( hFeature );
        }
    }
    
}

OGRGeometryH TIndexKernel::fetchGeometry(MetadataNode metadata)
{

    // fetch the boundary geometry out of the metadata
    // This will also fetch the SRS and assign it to the 
    // geometry.

    std::string wkt = metadata.findChild("boundary:boundary").value();
    std::string srs = metadata.findChild("summary:spatial_reference").value();
    std::string filename = metadata.findChild("filename").value();

    std::cout << "boundary is '" << wkt << "'"<<std::endl;
    std::cout << "srs is '" << srs << "'"<<std::endl;


    OGRGeometryH hGeometry(0);
    OGRSpatialReferenceH hSRS = OSRNewSpatialReference(NULL);

    char* p_wkt = (char*)wkt.c_str();
    char* p_srs = (char*)srs.c_str();
    OGRErr err = OSRImportFromWkt(hSRS, &p_srs);
    if (err != OGRERR_NONE)
    {
        std::cerr << "Unable to import SRS for file '" << filename << "'" << std::endl;
    }
    err = OGR_G_CreateFromWkt(&p_wkt, hSRS, &hGeometry);
    if (err != OGRERR_NONE)
    {
        std::cerr << "Unable to SRS for file '" << filename << "'" << std::endl;
    }

    return hGeometry;



}


int TIndexKernel::execute()
{
    GlobalEnvironment::get().getGDALEnvironment();
    m_kernelFactory = (void*) new KernelFactory (false);

    std::vector<std::string> files = glob(m_indexDirectory);

    std::cout <<" here " << std::endl;
    bool bCreatedLayer(false);
    for (auto f: files)
    {
        f = pdal::FileUtils::toAbsolutePath(f);
        MetadataNode m = fetchInfo(f);
        std::string wkt = m.findChild("boundary:boundary").value();
        std::string srs = m.findChild("summary:spatial_reference").value();

        if (!bCreatedLayer)
        {
            createDS(m_outputFilename);
            createLayer(m_outputFilename, srs);
            bCreatedLayer = true;
        }
        OGRGeometryH g = fetchGeometry(m);
        std::cout << "f: " << f << std::endl;
    }


    return 0;
}

} // namespace pdal

