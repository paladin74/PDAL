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

#include "GeoPackageManager.hpp"
#include "GeoPackageCommon.hpp"

#include <../plugins/sqlite/io/SQLiteCommon.hpp>
#include <../filters/tiler/TilerCommon.hpp>


namespace pdal
{

namespace rialto
{

GeoPackageManager::GeoPackageManager(const std::string& connection, LogPtr log) :
    GeoPackage(connection, log),
    e_creationOpen("creationOpen"),
    e_creationClose("creationClose")
{
}


GeoPackageManager::~GeoPackageManager()
{
}


void GeoPackageManager::open()
{
    e_creationOpen.start();

    internalOpen(true);

    if (!doesTableExist("gpkg_contents"))
    {
        createGpkgId();
        createTableGpkgSpatialRefSys();
        createTableGpkgContents();
        createTableGpkgPctileMatrixSet();
        createTableGpkgPctileMatrix();
        createTableGpkgMetadata();
        createTableGpkgMetadataReference();
        createTableGpkgExtensions();
        createTablePctilesDimensionSet();
    }

    e_creationOpen.stop();
}


void GeoPackageManager::close()
{
    e_creationClose.start();

    internalClose();

    e_creationClose.stop();

    dumpStats();
}


void GeoPackageManager::createGpkgId()
{
    const std::string sql("PRAGMA application_id=1196437808");

    m_sqlite->execute(sql);
}


void GeoPackageManager::createTableGpkgSpatialRefSys()
{
    if (m_sqlite->doesTableExist("gpkg_spatial_ref_sys"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_spatial_ref_sys' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_spatial_ref_sys("
        "srs_name TEXT NOT NULL,"
        "srs_id INTEGER PRIMARY KEY NOT NULL,"
        "organization TEXT NOT NULL,"
        "organization_coordsys_id INTEGER NOT NULL,"
        "definition TEXT NOT NULL,"
        "description TEXT"
        ")";

    m_sqlite->execute(sql);

    const std::string data =
        "INSERT INTO gpkg_spatial_ref_sys "
        "(srs_name, srs_id, organization, organization_coordsys_id, definition, description) "
        "VALUES (?, ?, ?, ?, ?, ?)";

    records rs1, rs2, rs3;
    row r1;
    row r2;
    row r3;

    const std::string wkt4326 = SpatialReference("EPSG:4326").getWKT(SpatialReference::eCompoundOK);
    r1.push_back(column("EPSG:4326"));
    r1.push_back(column(4326));
    r1.push_back(column("EPSG"));
    r1.push_back(column(4326));
    r1.push_back(column(wkt4326));
    r1.push_back(column("EPSG:4326"));
    rs1.push_back(r1);
    m_sqlite->insert(data, rs1);

    r2.push_back(column("undefined_cartesian"));
    r2.push_back(column(-1));
    r2.push_back(column("NONE"));
    r2.push_back(column(-1));
    r2.push_back(column("undefined"));
    r2.push_back(column("undefined_cartesian"));
    rs2.push_back(r2);
    m_sqlite->insert(data, rs2);

    r3.push_back(column("undefined_geographic"));
    r3.push_back(column(0));
    r3.push_back(column("NONE"));
    r3.push_back(column(0));
    r3.push_back(column("undefined"));
    r3.push_back(column("undefined_geographic"));
    rs3.push_back(r3);
    m_sqlite->insert(data, rs3);
}


void GeoPackageManager::createTableGpkgContents()
{
    if (m_sqlite->doesTableExist("gpkg_contents"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_contents' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_contents("
        "table_name TEXT PRIMARY KEY NOT NULL,"
        "data_type TEXT NOT NULL,"
        "identifier TEXT,"
        "description TEXT,"
        "last_change DATETIME NOT NULL,"
        "min_x DOUBLE NOT NULL," // data extents
        "min_y DOUBLE NOT NULL,"
        "max_x DOUBLE NOT NULL,"
        "max_y DOUBLE NOT NULL,"
        "srs_id INTEGER,"
        "FOREIGN KEY(srs_id) REFERENCES gpkg_spatial_ref_sys(srs_id)"
        ")";

    m_sqlite->execute(sql);
}


void GeoPackageManager::createTableGpkgPctileMatrixSet()
{
    if (m_sqlite->doesTableExist("gpkg_pctile_matrix_set"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_pctile_matrix_set' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_pctile_matrix_set("
        "table_name TEXT PRIMARY KEY NOT NULL,"
        "srs_id INTEGER NOT NULL,"
        "min_x DOUBLE NOT NULL," // tile extents
        "min_y DOUBLE NOT NULL,"
        "max_x DOUBLE NOT NULL,"
        "max_y DOUBLE NOT NULL,"
        "FOREIGN KEY(table_name) REFERENCES gpkg_contents(table_name)"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_metadata_reference(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(gpkg_pctile_dimension_set)"
        ")";

    m_sqlite->execute(sql);
}


void GeoPackageManager::createTableGpkgPctileMatrix()
{
    if (m_sqlite->doesTableExist("gpkg_pctile_matrix"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_pctile_matrix' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_pctile_matrix("
        "table_name TEXT NOT NULL,"
        "zoom_level INTEGER NOT NULL,"
        "matrix_width INTEGER NOT NULL,"
        "matrix_height INTEGER NOT NULL,"
        "FOREIGN KEY(table_name) REFERENCES gpkg_contents(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix_set(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_metadata_reference(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(gpkg_pctile_dimension_set)"
        ")";

    m_sqlite->execute(sql);
}

void GeoPackageManager::createTableGpkgMetadata()
{
    if (m_sqlite->doesTableExist("gpkg_metadata"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_metadata' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_metadata("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "md_scope TEXT NOT NULL,"
        "md_standard_uri TEXT NOT NULL,"
        "mime_type TEXT NOT NULL,"
        "metadata TEXT NOT NULL"
        ")";

    m_sqlite->execute(sql);
}


void GeoPackageManager::createTableGpkgMetadataReference()
{
    if (m_sqlite->doesTableExist("gpkg_metadata_reference"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_metadata_reference' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_metadata_reference("
        "reference_scope TEXT NOT NULL,"
        "table_name TEXT NOT NULL,"
        "column_name TEXT,"
        "row_id_value INTEGER,"
        "timestamp DATETIME NOT NULL,"
        "md_file_id INTEGER NOT NULL,"
        "md_parent_id INTEGER,"
        "FOREIGN KEY(table_name) REFERENCES gpkg_contents(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix_set(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(gpkg_pctile_dimension_set),"
        "FOREIGN KEY(md_file_id) REFERENCES gpkg_metadata(id),"
        "FOREIGN KEY(md_parent_id) REFERENCES gpkg_metadata(id)"
        ")";

    m_sqlite->execute(sql);
}


void GeoPackageManager::createTableGpkgExtensions()
{
    if (m_sqlite->doesTableExist("gpkg_extensions"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_extensions' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_extensions("
        "table_name TEXT,"
        "column_name TEXT,"
        "extension_name TEXT NOT NULL,"
        "definition TEXT NOT NULL,"
        "scope TEXT NOT NULL,"
        "FOREIGN KEY(table_name) REFERENCES gpkg_contents(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix_set(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_metadata_reference(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(gpkg_pctile_dimension_set),"
        "UNIQUE(table_name, column_name, extension_name)"
        ")";

    m_sqlite->execute(sql);
}


void GeoPackageManager::createTablePctilesDimensionSet()
{
    if (m_sqlite->doesTableExist("gpkg_pctile_dimension_set"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_pctile_dimension_set' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_pctile_dimension_set("
        "table_name TEXT NOT NULL,"
        "ordinal_position INTEGER NOT NULL,"
        "dimension_name TEXT NOT NULL,"
        "data_type TEXT NOT NULL,"
        "description TEXT,"
        "minimum DOUBLE,"
        "mean DOUBLE,"
        "maximum DOUBLE,"
        "FOREIGN KEY(table_name) REFERENCES gpkg_contents(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix_set(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_metadata_reference(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(table_name),"
        "UNIQUE(table_name, ordinal_position)"
        ")";

    m_sqlite->execute(sql);
}


void GeoPackageManager::childDumpStats() const
{
    std::cout << "GeoPackageManager stats" << std::endl;
    e_creationOpen.dump();
    e_creationClose.dump();
}


void GeoPackageManager::dropMatrixSet(const std::string& matrixSetName)
{
    // drops the matrix set table and all referencesto it  in the gpkg tables
    
    const std::vector<const std::string> tables {
        "gpkg_contents",
        "gpkg_pctile_dimension_set",
        "gpkg_pctile_matrix",
        "gpkg_pctile_matrix_set",
        "gpkg_metadata_reference",
        "gpkg_extensions"
    };
    
    for (auto table: tables) 
    {
        std::ostringstream oss2;
        oss2 << "DELETE FROM " << table << " WHERE table_name='" << matrixSetName << "'";
        m_sqlite->execute(oss2.str());
    }

    std::ostringstream oss;
    oss << "DROP TABLE IF EXISTS  " << matrixSetName;
    m_sqlite->execute(oss.str());    
}

} // namespace rialto
} // namespace pdal
