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
#include <time.h>
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
#include <gdal.h>

#include <boost/program_options.hpp>

namespace
{

void setDate(OGRFeatureH feature, const tm& tyme, int fieldNumber)
{
    OGR_F_SetFieldDateTime(feature, fieldNumber,
        tyme.tm_year + 1900, tyme.tm_mon + 1, tyme.tm_mday, tyme.tm_hour,
        tyme.tm_min, tyme.tm_sec, 100);
}


pdal::StringList glob(const std::string& pat)
{
    pdal::StringList filenames;

#ifndef WIN32
    glob_t glob_result;

    glob(pat.c_str(), GLOB_TILDE, NULL,& glob_result);
    for (unsigned int i = 0; i < glob_result.gl_pathc; ++i)
        filenames.push_back(std::string(glob_result.gl_pathv[i]));
    globfree(&glob_result);
#endif

    return filenames;
}

} // anonymous namespace


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
    , m_driverName("ESRI Shapefile")
    , m_tileIndexColumnName("location")
    , m_srsColumnName("srs")
    , m_merge(false)
    , m_dataset(NULL)
    , m_layer(NULL)

{
    m_log.setLeader("pdal tindex");
}


void TIndexKernel::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("filename", po::value<std::string>(&m_filename),
            "OGR-readable/writeable tile index output")
        ("directory", po::value<std::string>(&m_indexDirectory),
            "Directory to index")
        ("lyr_name", po::value<std::string>(&m_layerName),
            "OGR layer name to write into datasource")
        ("tile_index", po::value<std::string>(&m_tileIndexColumnName),
            "Tile index column name")
        ("src_srs_name", po::value<std::string>(&m_srsColumnName),
            "SRS column name")
        ("driver_name,f",
            po::value<std::string>(&m_driverName)->default_value(
            "ESRI Shapefile"), "OGR driver name to use ")
        ("t_srs", po::value<std::string>(&m_targetSRSString)->
            default_value("EPSG:4326"), "Target SRS of tile index")
        ("merge", "Whether we're merging the entries in a tindex file.")
    ;

    addSwitchSet(file_options);
    po::options_description* processing_options =
        new po::options_description("processing options");

    processing_options->add_options();

    addSwitchSet(processing_options);

    addPositionalSwitch("filename", 1);
    addPositionalSwitch("directory", 2);
}


void TIndexKernel::validateSwitches()
{
    m_merge = argumentExists("merge");

    if (!m_filename.size())
    {
        std::string err = m_merge ? "No input tindex filename provided." :
            "No output filename provided.";
        throw pdal_error(err);
    }
}


int TIndexKernel::execute()
{
    GlobalEnvironment::get().initializeGDAL(0);

    if (m_merge)
        ;
    else
        createFile();
    return 0;
}


void TIndexKernel::createFile()
{
    StringList files = glob(m_indexDirectory);

    if (files.empty())
    {
        std::ostringstream out;
        out << "Couldn't open or create index dataset file '" <<
            m_filename << "'.";
        throw pdal_error(out.str());
    }

    const std::string filename = files.front();
    if (m_layerName.empty())
       m_layerName = CPLGetBasename(filename.c_str());

    // Open or create the dataset.
    if (!openDataset(m_filename))
        if (!createDataset(m_filename))
        {
            std::ostringstream out;
            out << "Couldn't open or create index dataset file '" <<
                m_filename << "'.";
            throw pdal_error(out.str());
        }

    // Open or create a layer
    if (!openLayer(m_layerName))
        if (!createLayer(m_layerName))
        {
            std::ostringstream out;
            out << "Couldn't open or create layer '" << m_layerName <<
                "' in output file '" << m_filename << "'.";
            throw pdal_error(out.str());
        }

    FieldIndexes indexes = getFields();

    KernelFactory factory(false);
    for (auto f : files)
    {
        f = FileUtils::toAbsolutePath(f);
        FileInfo info = getFileInfo(factory, f);
        if (createFeature(indexes, info))
            m_log.get(LogLevel::Info) << "Indexed file " << f << std::endl;
        else
            m_log.get(LogLevel::Error) << "Failed to create feature for "
                "file '" << f << "'" << std::endl;
    }
}


bool TIndexKernel::createFeature(const FieldIndexes& indexes,
    const FileInfo& fileInfo)
{
    OGRFeatureH hFeature = OGR_F_Create(OGR_L_GetLayerDefn(m_layer));

    // Set the creation time into the feature.
    setDate(hFeature, fileInfo.m_ctime, indexes.m_ctime);

    // Set the file mod time into the feature.
    setDate(hFeature, fileInfo.m_mtime, indexes.m_mtime);

    // Set the filename into the feature.
    OGR_F_SetFieldString(hFeature, indexes.m_filename,
        fileInfo.m_filename.c_str());

    // Set the SRS into the feature.
    OGRSpatialReferenceH hSourceSRS = OSRNewSpatialReference("");
    std::string srs = fileInfo.m_srs;

    if (OSRSetFromUserInput(hSourceSRS, srs.c_str()) != OGRERR_NONE)
    {
        m_log.get(LogLevel::Error) << "Unable to import spatial "
            "reference '" << fileInfo.m_srs << "' for file '" <<
            fileInfo.m_filename << "'" << std::endl;
    }

    // We have a limit of like 254 characters in some formats (notably
    // shapefile), so try to get the condensed version of the SRS.

    // Failing that, get the proj.4 version.  Not sure what's supposed to
    // happen if we overflow 254 with proj.4.

    const char* pszAuthorityCode = OSRGetAuthorityCode(hSourceSRS, NULL);
    const char* pszAuthorityName = OSRGetAuthorityName(hSourceSRS, NULL);
    if (pszAuthorityName && pszAuthorityCode)
    {
        std::string auth = std::string(pszAuthorityName) + ":" +
            pszAuthorityCode;
        OGR_F_SetFieldString(hFeature, indexes.m_srs, auth.data());
    }
    else
    {
        char* pszProj4 = NULL;
        if (OSRExportToProj4(hSourceSRS, &pszProj4) != OGRERR_NONE)
        {
            std::ostringstream out;

            out << "Unable to output proj4 SRS for file '" <<
                fileInfo.m_filename << "'";
            throw pdal_error(out.str());
        }
        srs = std::string(pszProj4);
        OGR_F_SetFieldString(hFeature, indexes.m_srs, srs.c_str());
        CPLFree(pszProj4);
    }

    OSRDestroySpatialReference(hSourceSRS);

    // Set the geometry in the feature
    OGRGeometryH g = prepareGeometry(fileInfo);
    OGR_F_SetGeometry(hFeature, g);
    OGR_G_DestroyGeometry(g);

    bool ok = (OGR_L_CreateFeature(m_layer, hFeature) != OGRERR_NONE);

    OGR_F_Destroy(hFeature);
    return ok;
}


TIndexKernel::FileInfo TIndexKernel::getFileInfo(KernelFactory& factory,
    const std::string& filename)
{
    FileInfo fileInfo;

    std::unique_ptr<Kernel> app = factory.createKernel("kernels.info");
    InfoKernel *info = static_cast<InfoKernel *>(app.get());
    
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

    fileInfo.m_boundary = metadata.findChild("boundary:boundary").value();
    fileInfo.m_srs = metadata.findChild("summary:spatial_reference").value();

    //ABELL - Not portable.
    struct stat statbuf;
    stat(filename.c_str(), &statbuf);
    gmtime_r(&statbuf.st_mtime, &fileInfo.m_mtime);
    gmtime_r(&statbuf.st_mtime, &fileInfo.m_ctime);

    return fileInfo;
}


bool TIndexKernel::openDataset(const std::string& filename)
{
    m_dataset = OGROpen(filename.c_str(), TRUE, NULL);
    return (bool)m_dataset;
}


bool TIndexKernel::createDataset(const std::string& filename)
{
    OGRSFDriverH hDriver = OGRGetDriverByName(m_driverName.c_str());
    if (!hDriver)
    {
        std::ostringstream oss;

        oss << "Can't create dataset using driver '" << m_driverName <<
            "'. Driver is not available.";
        throw pdal_error(oss.str());
    }

    std::string dsname = FileUtils::toAbsolutePath(filename);
    m_dataset = OGR_Dr_CreateDataSource(hDriver, dsname.c_str(), NULL);
    return (bool)m_dataset;
}


bool TIndexKernel::openLayer(const std::string& layerName)
{
    if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);
    else if (layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, m_layerName.c_str());

    return (bool)m_layer;
}


bool TIndexKernel::createLayer(std::string const& layername)
{
    OGRSpatialReferenceH hSpatialRef = OSRNewSpatialReference("");

    if (OSRSetFromUserInput(hSpatialRef,
        m_targetSRSString.c_str()) != OGRERR_NONE)
    {
        m_log.get(LogLevel::Error) << "Unable to import srs for layer "
           "creation" << std::endl;
    }
    m_layer = OGR_DS_CreateLayer(m_dataset, m_layerName.c_str(),
        hSpatialRef, wkbPolygon, NULL);
    OSRRelease(hSpatialRef);

    if (m_layer)
        createFields();
    return (bool)m_layer;
}


void TIndexKernel::createFields()
{
    OGRFieldDefnH hFieldDefn = OGR_Fld_Create(
        m_tileIndexColumnName.c_str(), OFTString);
    OGR_Fld_SetWidth(hFieldDefn, 254);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create(m_srsColumnName.c_str(), OFTString);
    OGR_Fld_SetWidth(hFieldDefn, 254);
    OGR_L_CreateField( m_layer, hFieldDefn, TRUE );
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create("modified", OFTDateTime);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create("created", OFTDateTime);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);

    OGR_L_SyncToDisk(m_layer);
}


TIndexKernel::FieldIndexes TIndexKernel::getFields()
{
    FieldIndexes indexes;

    void *fDefn = OGR_L_GetLayerDefn(m_layer);
    indexes.m_filename = OGR_FD_GetFieldIndex(fDefn,
        m_tileIndexColumnName.c_str());
    if (indexes.m_filename < 0)
    {
        std::ostringstream out;

        out << "Unable to find field '" << m_tileIndexColumnName <<
            "' in file '" << m_filename << "'.";
        throw pdal_error(out.str());
    }
    indexes.m_srs = OGR_FD_GetFieldIndex(fDefn, m_srsColumnName.c_str());
    if (indexes.m_srs < 0)
    {
        std::ostringstream out;

        out << "Unable to find field '" << m_srsColumnName << "' in file '" <<
            m_filename << "'.";
        throw pdal_error(out.str());
    }

    indexes.m_ctime = OGR_FD_GetFieldIndex(fDefn, "created");
    indexes.m_mtime = OGR_FD_GetFieldIndex(fDefn, "modified");

    /* Load in memory existing file names in SHP */
    int nExistingFiles = (int)OGR_L_GetFeatureCount(m_layer, FALSE);
    for (auto i = 0; i < nExistingFiles; i++)
    {
        OGRFeatureH hFeature = OGR_L_GetNextFeature(m_layer);
        m_files.push_back(OGR_F_GetFieldAsString(hFeature, indexes.m_filename));
        OGR_F_Destroy(hFeature);
    }
    return indexes;
}


OGRGeometryH TIndexKernel::prepareGeometry(const FileInfo& fileInfo)
{
    // Create OGR geometry from text.

    OGRGeometryH hGeometry(0);
    OGRSpatialReferenceH hSRS = OSRNewSpatialReference("");

    char* p_srs = const_cast<char *>(fileInfo.m_srs.data());
    if (OSRImportFromWkt(hSRS, &p_srs) != OGRERR_NONE)
        m_log.get(LogLevel::Error) << "Unable to import SRS for file '" <<
            fileInfo.m_filename << "'" << std::endl;

    char* p_wkt = const_cast<char *>(fileInfo.m_boundary.data());
    if (OGR_G_CreateFromWkt(&p_wkt, hSRS, &hGeometry) != OGRERR_NONE)
        m_log.get(LogLevel::Error) << "Unable to create WKT for file '" <<
            fileInfo.m_filename << "'" << std::endl;

    OGRSpatialReferenceH hTargetSRS = OSRNewSpatialReference("");
    if (OSRSetFromUserInput(hTargetSRS, m_targetSRSString.c_str()) !=
            OGRERR_NONE)
        m_log.get(LogLevel::Error) << "Unable to import SRS for "
            "--t_srs '" << m_targetSRSString << "'" << std::endl;

    if (OGR_G_TransformTo(hGeometry, hTargetSRS) != OGRERR_NONE)
        m_log.get(LogLevel::Error) << "Unable to transform geometry to "
            "output srs" << std::endl;

    OSRDestroySpatialReference(hTargetSRS);
    OSRDestroySpatialReference(hSRS);

    return hGeometry;
}

} // namespace pdal

