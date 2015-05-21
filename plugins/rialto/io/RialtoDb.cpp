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

#include "RialtoDb.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_error.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include <cstdint>

#include <../plugins/sqlite/io/SQLiteCommon.hpp> // TODO: fix path


namespace pdal
{

namespace
{

} // anonymous namespace


RialtoDb::RialtoDb(const std::string& connection, LogPtr log) :
    m_connection(connection),
    m_log(log),
    m_srid(4326),
    m_needsIndexing(false),
    m_txStarted(false),
    e_tilesRead("tilesRead"),
    e_tileSetsRead("tileSetsRead"),
    e_tilesWritten("tilesWritten"),
    e_tileSetsWritten("tileSetsWritten"),
    e_queries("queries"),
    e_creation("creation"),
    e_indexCreation("indexCreation"),
    m_numPointsRead(0),
    m_numPointsWritten(0)
{
    //m_log->setLevel(LogLevel::Debug);
}


RialtoDb::~RialtoDb()
{
    if (m_sqlite)
    {
        close();
    }

    log()->get(LogLevel::Debug) << "~RialtoDB" << std::endl;
}


void RialtoDb::create()
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    log()->get(LogLevel::Debug) << "RialtoDB::create()" << std::endl;

    if (FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: database already exists");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(true);

    e_creation.start();
    
    createGpkgId();
    createTableGpkgSpatialRefSys();
    createTableGpkgContents();
    createTableGpkgPctileMatrixSet();
    createTableGpkgPctileMatrix();
    createTableGpkgMetadata();
    createTableGpkgMetadataReference();
    createTableGpkgExtensions();
    createTablePctilesDimensionSet();

    e_creation.stop();

    m_needsIndexing = true;
}


void RialtoDb::verifyTableExists(std::string const& name) const
{
    if (!m_sqlite->doesTableExist(name))
    {
        throw pdal_error("RialtoDb: required table '" + name + "' not found");
    }
}


void RialtoDb::open(bool writable)
{
    if (m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session already exists)");
    }

    log()->get(LogLevel::Debug) << "RialtoDB::open()" << std::endl;

    if (!FileUtils::fileExists(m_connection))
    {
      throw pdal_error("RialtoDb: database not found");
    }

    m_sqlite = std::unique_ptr<SQLite>(new SQLite(m_connection, m_log));
    m_sqlite->connect(writable);

    verifyTableExists("gpkg_spatial_ref_sys");
    verifyTableExists("gpkg_contents");
    verifyTableExists("gpkg_pctile_matrix");
    verifyTableExists("gpkg_pctile_matrix_set");
    verifyTableExists("gpkg_extensions");
    verifyTableExists("gpkg_metadata");
    verifyTableExists("gpkg_metadata_reference");
}


void RialtoDb::close()
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    if (m_needsIndexing)
    {
        e_indexCreation.start();

        if (m_txStarted)
        {
            m_sqlite->commit();
        }

#if 0
// TODO
        std::ostringstream oss2;
        oss2 << "CREATE INDEX index_name ON Tiles(column,row)";
        m_sqlite->execute(oss2.str());

        std::ostringstream oss3;
        oss3 << "CREATE INDEX index_name2 ON Tiles(level)";
        m_sqlite->execute(oss3.str());
#endif

        e_indexCreation.stop();
    }

    m_sqlite.reset();

    dumpStats();
}


void RialtoDb::createGpkgId()
{
    std::ostringstream oss1;

    const std::string sql("PRAGMA application_id=1196437808");

    m_sqlite->execute(sql);
}


void RialtoDb::createTableGpkgSpatialRefSys()
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


    r1.push_back(column("EPSG:4326"));
    r1.push_back(column(4326));
    r1.push_back(column("EPSG"));
    r1.push_back(column(4326));
    r1.push_back(column("EPSG:4326"));
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


void RialtoDb::createTableGpkgContents()
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
        "data_min_x DOUBLE NOT NULL," // data extents
        "data_min_y DOUBLE NOT NULL,"
        "data_max_x DOUBLE NOT NULL,"
        "data_max_y DOUBLE NOT NULL,"
        "srs_id INTEGER,"
        "FOREIGN KEY(srs_id) REFERENCES gpkg_spatial_ref_sys(srs_id)"
        ")";

    m_sqlite->execute(sql);
}


void RialtoDb::createTableGpkgPctileMatrixSet()
{
    if (m_sqlite->doesTableExist("gpkg_pctile_matrix_set"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'gpkg_pctile_matrix_set' already exists)");
    }

    const std::string sql =
        "CREATE TABLE gpkg_pctile_matrix_set("
        "table_name TEXT PRIMARY KEY NOT NULL,"
        "srs_id INTEGER NOT NULL,"
        "tmset_min_x DOUBLE NOT NULL," // tile extents
        "tmset_min_y DOUBLE NOT NULL,"
        "tmset_max_x DOUBLE NOT NULL,"
        "tmset_max_y DOUBLE NOT NULL,"
        "FOREIGN KEY(table_name) REFERENCES gpkg_contents(table_name)"
        "FOREIGN KEY(table_name) REFERENCES gpkg_pctile_matrix(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_metadata_reference(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(table_name),"
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(pctiles_dimension_set)"
        ")";

    m_sqlite->execute(sql);
}


void RialtoDb::createTableGpkgPctileMatrix()
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
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(pctiles_dimension_set)"
        ")";

    m_sqlite->execute(sql);
    
    for (int i=0; i<32; i++)
    {
        m_haveWrittenMatrixAtLevel[i] = false;
    }
}


void RialtoDb::createTableTilePyramidUserData(const std::string& table_name)
{
    if (m_sqlite->doesTableExist(table_name))
    {
        throw pdal_error("RialtoDB: invalid state (table '" + table_name + "' already exists)");
    }

    const std::string sql =
        "CREATE TABLE " + table_name + "("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "zoom_level INTEGER NOT NULL,"
        "tile_column INTEGER NOT NULL,"
        "tile_row INTEGER NOT NULL,"
        "tile_data BLOB NOT NULL,"
        "num_points INTEGER NOT NULL,"
        "child_mask INTEGER NOT NULL,"
        "UNIQUE(zoom_level, tile_column, tile_row)"
        ")";

    m_sqlite->execute(sql);
    
    const std::string data =
        "INSERT INTO gpkg_extensions "
        "(table_name, column_name, extension_name, definition, scope) "
        "VALUES (?, ?, ?, ?, ?)";

    records rs;
    row r;

    r.push_back(column(table_name));
    r.push_back(column("NULL"));
    r.push_back(column("radiantblue_pctiles"));
    r.push_back(column("mailto:mpg@flaxen.com"));
    r.push_back(column("read-write"));
    rs.push_back(r);

    m_sqlite->insert(data, rs);
}


void RialtoDb::createTableGpkgMetadata()
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


void RialtoDb::createTableGpkgMetadataReference()
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
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(pctiles_dimension_set),"
        "FOREIGN KEY(md_file_id) REFERENCES gpkg_metadata(id),"
        "FOREIGN KEY(md_parent_id) REFERENCES gpkg_metadata(id)"
        ")";

    m_sqlite->execute(sql);
}


void RialtoDb::createTableGpkgExtensions()
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
        "FOREIGN KEY(table_name) REFERENCES gpkg_extensions(pctiles_dimension_set),"
        "UNIQUE(table_name, column_name, extension_name)"
        ")";

    m_sqlite->execute(sql);
}


void RialtoDb::createTablePctilesDimensionSet()
{
    if (m_sqlite->doesTableExist("pctiles_dimension_set"))
    {
        throw pdal_error("RialtoDB: invalid state (table 'pctiles_dimension_set' already exists)");
    }

    const std::string sql =
        "CREATE TABLE pctiles_dimension_set("
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


void RialtoDb::readTileSetIds(std::vector<std::string>& names) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }
    
    names.clear();

    std::ostringstream oss;
    oss << "SELECT table_name FROM gpkg_contents";

    log()->get(LogLevel::Debug1) << "SELECT for tile set ids" << std::endl;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        column const& c = r->at(0);
        const std::string name = c.data;
        log()->get(LogLevel::Debug1) << " got name: " << name << std::endl;
        names.push_back(name);

    } while (m_sqlite->next());
}    


void RialtoDb::readTileSetInfo(std::string const& name, TileSetInfo& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tileSetsRead.start();

    info.name = name;

    {
        std::ostringstream oss;
        oss << "SELECT table_name "
            << "FROM gpkg_pctile_matrix_set WHERE table_name='" << name << "'";

        log()->get(LogLevel::Debug) << "SELECT for tile set" << std::endl;

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        assert(name == r->at(0).data);
        assert(!m_sqlite->next());
    }
    
    {
        std::ostringstream oss;
        oss << "SELECT MAX(zoom_level) "
            << "FROM gpkg_pctile_matrix WHERE table_name='" << name << "'";

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        info.maxLevel = boost::lexical_cast<uint32_t>(r->at(0).data);
        assert(!m_sqlite->next());
    }

    {
        std::ostringstream oss;
        oss << "SELECT COUNT(table_name) "
            << "FROM pctiles_dimension_set WHERE table_name='" << name << "'";

        m_sqlite->query(oss.str());

        // should get exactly one row back
        const row* r = m_sqlite->get();
        assert(r);
        info.numDimensions = boost::lexical_cast<uint32_t>(r->at(0).data);
        assert(info.numDimensions != 0);
        assert(!m_sqlite->next());
    }

    readDimensionsInfo(info.name, info.dimensions);
    assert(info.dimensions.size() == info.numDimensions);

    e_tileSetsRead.stop();
}


void RialtoDb::readTileInfo(std::string const& name, uint32_t tileId, bool withPoints, RialtoDb::TileInfo& info) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesRead.start();

    std::ostringstream oss;
    oss << "SELECT zoom_level,tile_column,tile_row,num_points"
        << (withPoints ? ",tile_data " : " ")
        << "FROM '" << name << "'"
        << "WHERE id=" << tileId;

    //log()->get(LogLevel::Debug) << "SELECT for tile" << std::endl;

    m_sqlite->query(oss.str());

    // should get exactly one row back
    const row* r = m_sqlite->get();
    assert(r);

    info.level = boost::lexical_cast<double>(r->at(0).data);
    info.column = boost::lexical_cast<double>(r->at(1).data);
    info.row = boost::lexical_cast<double>(r->at(2).data);
    info.numPoints = boost::lexical_cast<double>(r->at(3).data);

    info.patch.buf.clear();
    if (withPoints)
    {
      const uint32_t blobLen = r->at(4).blobLen;
      const std::vector<uint8_t>& blobBuf = r->at(4).blobBuf;
      const unsigned char *pos = (const unsigned char *)&(blobBuf[0]);
      info.patch.putBytes(pos, blobLen);

      ++m_numPointsRead;
    }

    e_tilesRead.stop();

    assert(!m_sqlite->next());
}


void RialtoDb::readTileIdsAtLevel(std::string const& name, uint32_t level, std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    ids.clear();

    std::ostringstream oss;
    oss << "SELECT id FROM '" << name << "'"
        << " WHERE zoom_level=" << level;

    log()->get(LogLevel::Debug) << "SELECT for tile ids at level: " << level << std::endl;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        //log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_sqlite->next());
}


void RialtoDb::readDimensionsInfo(std::string const& name, std::vector<DimensionInfo>& dimensionsInfo) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    dimensionsInfo.clear();

    std::ostringstream oss;
    oss << "SELECT ordinal_position, dimension_name, data_type, description, minimum, mean, maximum "
        << "FROM pctiles_dimension_set WHERE table_name='" << name << "'";

    m_sqlite->query(oss.str());

    int i = 0;
    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        DimensionInfo info;

        info.position = boost::lexical_cast<uint32_t>(r->at(0).data);
        info.name = r->at(1).data;
        info.dataType = r->at(2).data;
        info.description = r->at(3).data;
        info.minimum = boost::lexical_cast<double>(r->at(4).data);
        info.mean = boost::lexical_cast<double>(r->at(5).data);
        info.maximum = boost::lexical_cast<double>(r->at(6).data);

        log()->get(LogLevel::Debug1) << "read dim: " << info.name << std::endl;

        ++i;
        
        dimensionsInfo.push_back(info);
    } while (m_sqlite->next());
}


void RialtoDb::writeTileSet(const RialtoDb::TileSetInfo& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "RialtoDb::addTileSet()" << std::endl;

    e_tileSetsWritten.start();

    {
        createTableTilePyramidUserData(data.name);
    }
    
    {
        const std::string sql =
            "INSERT INTO gpkg_contents"
            " (table_name, data_type, identifier, description, last_change,"
            " data_min_x, data_min_y, data_max_x, data_max_y, srs_id)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column(data.name)); // table_name
        r.push_back(column("pctiles")); // data_type
        r.push_back(column(data.name)); // identifier
        r.push_back(column(data.name)); // description
        r.push_back(column(data.datetime)); // last_change
        r.push_back(column(4326)); // srs_id
        r.push_back(column(data.data_min_x));
        r.push_back(column(data.data_min_y));
        r.push_back(column(data.data_max_x));
        r.push_back(column(data.data_max_y));

        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }
    
    {
        const std::string sql =
            "INSERT INTO gpkg_pctile_matrix_set"
            " (table_name, srs_id,"
            " tmset_min_x, tmset_min_y, tmset_max_x, tmset_max_y)"
            " VALUES (?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column(data.name)); // table_name
        r.push_back(column(4326)); // srs_id
        r.push_back(column(data.tmset_min_x));
        r.push_back(column(data.tmset_min_y));
        r.push_back(column(data.tmset_max_x));
        r.push_back(column(data.tmset_max_y));

        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }
    
    {
        const std::string sql =
            "INSERT INTO pctiles_dimension_set"
            " (table_name, dimension_name, data_type, ordinal_position, description,"
            " minimum, mean, maximum)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?)";
        
        int i = 0;
        for (auto dim: data.dimensions)
        {
            records rs;
            row r;
            
            r.push_back(column(data.name)); // table_name
            r.push_back(column(dim.name.c_str()));
            r.push_back(column(dim.dataType));
            r.push_back(column(i)); // ordinal_position
            r.push_back(column("...description..."));
            r.push_back(column(dim.minimum));
            r.push_back(column(dim.mean));
            r.push_back(column(dim.maximum));

            rs.push_back(r);

            m_sqlite->insert(sql, rs);
            
            ++i;
        }
    }

    uint32_t metadata_id;
    {
        const std::string sql =
            "INSERT INTO gpkg_metadata"
            " (md_scope, md_standard_uri, mime_type, metadata)"
            " VALUES (?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column("dataset"));
        r.push_back(column("LAS")); // TODO
        r.push_back(column("text/xml")); // TODO
        r.push_back(column("...data..."));

        rs.push_back(r);
        
        m_sqlite->insert(sql, rs);        

        metadata_id = m_sqlite->last_row_id();
    }

    {
        const std::string sql =
            "INSERT INTO gpkg_metadata_reference"
            " (reference_scope, table_name, column_name, row_id_value, timestamp, md_file_id, md_parent_id)"
            " VALUES (?, ?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column("table")); // reference_scope
        r.push_back(column(data.name)); // table_name
        r.push_back(column("table")); // column_name
        r.push_back(column("NULL")); // row_id_value
        r.push_back(column(data.datetime)); // timestamp
        r.push_back(column(metadata_id)); // md_file_id
        r.push_back(column("NULL")); // md_parent_id

        rs.push_back(r);
        
        m_sqlite->insert(sql, rs);        
    }

    e_tileSetsWritten.stop();
}


void RialtoDb::writeDimensions(uint32_t tileSetId,
                             const std::vector<DimensionInfo>& dims)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    std::ostringstream oss;
    oss << "INSERT INTO Dimensions "
      << "(tile_set_id, name, position, dataType, minimum, mean, maximum) "
      << "VALUES (?, ?, ?, ?, ?, ?, ?)";

    for (auto dim: dims)
    {
        records rs;
        row r;

        log()->get(LogLevel::Debug) << "INSERT for dim: " << dim.name << std::endl;

        r.push_back(column(tileSetId));
        r.push_back(column(dim.name.c_str()));
        r.push_back(column(dim.position));
        r.push_back(column(dim.dataType));
        r.push_back(column(dim.minimum));
        r.push_back(column(dim.mean));
        r.push_back(column(dim.maximum));
        rs.push_back(r);

        m_sqlite->insert(oss.str(), rs);
    }
}


void RialtoDb::writeTile(const std::string& tileSetName, const RialtoDb::TileInfo& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesWritten.start();

    if (!m_txStarted)
    {
        m_sqlite->begin();
        m_txStarted = true;
    }

    unsigned char* buf = NULL;
    uint32_t buflen = 0;
    castPatchAsBuffer(data.patch, buf, buflen);
    assert(buf);
    assert(buflen);

    if (!m_haveWrittenMatrixAtLevel[data.level])
    {
        const std::string sql =
            "INSERT INTO gpkg_pctile_matrix"
            " (table_name, zoom_level, matrix_width, matrix_height)"
            " VALUES (?, ?, ?, ?)";

        records rs;
        row r;


        uint32_t numCols, numRows;
        matrixSizeAtLevel(data.level, numCols, numRows);

        r.push_back(column(tileSetName));
        r.push_back(column(data.level));
        r.push_back(column(numCols));
        r.push_back(column(numRows));
        rs.push_back(r);

        m_sqlite->insert(sql, rs);
        
        m_haveWrittenMatrixAtLevel[data.level] = true;
    }

    {
        const std::string sql =
            "INSERT INTO " + tileSetName +
            " (zoom_level, tile_column, tile_row, tile_data, num_points, child_mask)"
            " VALUES (?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column(data.level));
        r.push_back(column(data.column));
        r.push_back(column(data.row));
        r.push_back(blob((char*)buf, (size_t)buflen));
        r.push_back(column(data.numPoints));
        r.push_back(column(data.mask));
        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }

    e_tilesWritten.stop();

    m_numPointsWritten += data.numPoints;
}


void RialtoDb::castPatchAsBuffer(const Patch& patch, unsigned char*& buf, uint32_t& bufLen)
{
    buf = NULL;
    bufLen = patch.buf.size();
    if (bufLen) {
        buf = (unsigned char*)&patch.buf[0];
    }
}


void RialtoDb::xyPointToTileColRow(double x, double y, uint32_t level, uint32_t& col, uint32_t& row)
{
    if (x>=180.0) x = -180.0;
    if (y<=-90.0) y = 90.0;

    const double level2 = pow(2.0, level);

    double tileWidth = (180.0 - -180.0) / level2;
    tileWidth /= 2.0;
    const double tileHeight = (90.0 - -90.0) / level2;

    const double c = (x - -180.0) / tileWidth;
    col = (uint32_t)floor(c);

    const double r = (90.0 - y) / tileHeight;
    row = (uint32_t)floor(r);
}


void RialtoDb::matrixSizeAtLevel(uint32_t level, uint32_t& numCols, uint32_t& numRows) const
{
    const uint32_t level2 = (uint32_t)pow(2.0, level);

    numRows = level2;
    numCols = level2 * 2;
}


void RialtoDb::queryForTileIds(std::string const& name,
                               double minx, double miny,
                               double maxx, double maxy,
                               uint32_t level,
                               std::vector<uint32_t>& ids) const
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "Querying tile set " << name
                                << " for some tile ids" << std::endl;

    ids.clear();

    e_queries.start();

    assert(minx <= maxx);
    assert(miny <= maxy);

    uint32_t mincol, minrow, maxcol, maxrow;
    xyPointToTileColRow(minx, miny, level, mincol, maxrow);
    xyPointToTileColRow(maxx, maxy, level, maxcol, minrow);

    // because boundary conditions
    if (mincol > maxcol) std::swap(mincol, maxcol);
    if (minrow > maxrow) std::swap(minrow, maxrow);

    assert(mincol <= maxcol);
    assert(minrow <= maxrow);

    std::ostringstream oss;
    oss << "SELECT id FROM '" << name << "'"
        << " WHERE zoom_level=" << level
        << " AND tile_column >= " << mincol
        << " AND tile_column <= " << maxcol
        << " AND tile_row >= " << minrow
        << " AND tile_row <= " << maxrow;

    m_sqlite->query(oss.str());

    do {
        const row* r = m_sqlite->get();
        if (!r) break;

        uint32_t id = boost::lexical_cast<uint32_t>(r->at(0).data);
        log()->get(LogLevel::Debug) << "  got tile id=" << id << std::endl;
        ids.push_back(id);
    } while (m_sqlite->next());

    e_queries.stop();
}


void RialtoDb::queryForTileInfosBegin(std::string const& name,
                                      double minx, double miny,
                                      double maxx, double maxy,
                                      uint32_t level)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesRead.start();

    log()->get(LogLevel::Debug) << "Querying tile set " << name
                                << " for some tile infos" << std::endl;

    assert(minx <= maxx);
    assert(miny <= maxy);

    uint32_t mincol, minrow, maxcol, maxrow;
    xyPointToTileColRow(minx, miny, level, mincol, maxrow);
    xyPointToTileColRow(maxx, maxy, level, maxcol, minrow);

    // because boundary conditions
    if (mincol > maxcol) std::swap(mincol, maxcol);
    if (minrow > maxrow) std::swap(minrow, maxrow);

    assert(mincol <= maxcol);
    assert(minrow <= maxrow);

    std::ostringstream oss;
    oss << "SELECT zoom_level,tile_column,tile_row,num_points,tile_data"
        << " FROM '" << name << "'"
        << " WHERE zoom_level=" << level
        << " AND tile_column >= " << mincol
        << " AND tile_column <= " << maxcol
        << " AND tile_row >= " << minrow
        << " AND tile_row <= " << maxrow;

    m_sqlite->query(oss.str());

    e_tilesRead.stop();
}


bool RialtoDb::queryForTileInfos(TileInfo& info)
{
    e_tilesRead.start();

    const row* r = m_sqlite->get();

    if (!r) 
    {
        e_tilesRead.stop();
        return false;
    }

    info.level = boost::lexical_cast<double>(r->at(0).data);
    info.column = boost::lexical_cast<double>(r->at(1).data);
    info.row = boost::lexical_cast<double>(r->at(2).data);
    info.numPoints = boost::lexical_cast<double>(r->at(3).data);

    // this query always reads the points
    info.patch.buf.clear();
    {
        const uint32_t blobLen = r->at(4).blobLen;
        const std::vector<uint8_t>& blobBuf = r->at(4).blobBuf;
        const unsigned char *pos = (const unsigned char *)&(blobBuf[0]);
        info.patch.putBytes(pos, blobLen);
    }

    e_tilesRead.stop();

    m_numPointsRead += info.numPoints;

    return true;
}


bool RialtoDb::queryForTileInfosNext()
{
    return m_sqlite->next();
}


// appends points to end of view (does not start with point index 0)
void RialtoDb::serializeToPointView(const TileInfo& info, PointViewPtr view)
{
    const size_t numPoints = info.numPoints;
    PointId idx = view->size();
    const uint32_t pointSize = view->pointSize();

    const Patch& patch = info.patch;

    const char* buf = (const char*)(&patch.buf[0]);
    const DimTypeList& dtl = view->dimTypes();
    for (size_t i=0; i<numPoints; ++i)
    {
        view->setPackedPoint(dtl, idx, buf);
        buf += pointSize;
        ++idx;
    }
}


void RialtoDb::setupLayout(const TileSetInfo& tileSetInfo, PointLayoutPtr layout) const
{
    for (uint32_t i=0; i<tileSetInfo.numDimensions; i++)
    {
        const DimensionInfo& dimInfo = tileSetInfo.dimensions[i];

        const Dimension::Id::Enum nameId = Dimension::id(dimInfo.name);
        const Dimension::Type::Enum typeId = Dimension::type(dimInfo.dataType);

        layout->registerDim(nameId, typeId);
    }
}


void RialtoDb::dumpStats() const
{
    e_tilesRead.dump();
    e_tileSetsRead.dump();
    e_tilesWritten.dump();
    e_tileSetsWritten.dump();
    e_queries.dump();
    e_creation.dump();
    e_indexCreation.dump();

    if (m_numPointsRead)
    {
        printf("pointsRead: %u\n", m_numPointsRead);
    }
    else
    {
        printf("pointsRead: -\n");
    }

    if (m_numPointsWritten)
    {
        printf("pointsWritten: %u\n", m_numPointsWritten);
    }
    else
    {
      printf("pointsWritten: -\n");    
    }
}

} // namespace pdal
