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

#include "GeoPackageWriter.hpp"
#include "GeoPackageCommon.hpp"

#include <../plugins/sqlite/io/SQLiteCommon.hpp>
#include <../filters/tiler/TilerCommon.hpp>


namespace pdal
{

namespace rialto
{


GeoPackageWriter::GeoPackageWriter(const std::string& connection, LogPtr log) :
    GeoPackage(connection, log),
    m_srid(4326),
    m_needsIndexing(false),
    e_tilesWritten("tilesWritten"),
    e_tileTablesWritten("tileTablesWritten"),
    e_queries("queries"),
    m_numPointsWritten(0)
{
}


GeoPackageWriter::~GeoPackageWriter()
{
}


void GeoPackageWriter::open()
{
    internalOpen(true);

    verifyTableExists("gpkg_spatial_ref_sys");
    verifyTableExists("gpkg_contents");
    verifyTableExists("gpkg_pctile_matrix");
    verifyTableExists("gpkg_pctile_matrix_set");
    verifyTableExists("gpkg_extensions");
    verifyTableExists("gpkg_metadata");
    verifyTableExists("gpkg_metadata_reference");
    verifyTableExists("gpkg_pctile_dimension_set");
}


void GeoPackageWriter::close()
{
    internalClose();
    dumpStats();
}


void GeoPackageWriter::createTableGpkgPctile(const std::string& table_name)
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


void GeoPackageWriter::writeTileTable(const GpkgMatrixSet& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    log()->get(LogLevel::Debug) << "RialtoDb::addTileTable()" << std::endl;

    e_tileTablesWritten.start();

    assert(!m_sqlite->doesTableExist(data.getName()));
    createTableGpkgPctile(data.getName());
    assert(m_sqlite->doesTableExist(data.getName()));

    const uint32_t srs_id = querySrsId(data.getWkt());

    {
        // note min_x, etc, is the bbox of the DATA and not the tile matrix set
        const std::string sql =
            "INSERT INTO gpkg_contents"
            " (table_name, data_type, identifier, description, last_change, srs_id,"
            " min_x, min_y, max_x, max_y)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column(data.getName())); // table_name
        r.push_back(column("pctiles")); // data_type
        r.push_back(column(data.getName())); // identifier
        r.push_back(column(data.getDescription())); // description
        r.push_back(column(data.getDateTime())); // last_change
        r.push_back(column(srs_id));
        r.push_back(column(data.getDataMinX()));
        r.push_back(column(data.getDataMinY()));
        r.push_back(column(data.getDataMaxX()));
        r.push_back(column(data.getDataMaxY()));

        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }

    {
        // min_x, etc, is bbox of tile matrix (not data)
        const std::string sql =
            "INSERT INTO gpkg_pctile_matrix_set"
            " (table_name, srs_id,"
            " min_x, min_y, max_x, max_y)"
            " VALUES (?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column(data.getName())); // table_name
        r.push_back(column(srs_id));
        r.push_back(column(data.getTmsetMinX()));
        r.push_back(column(data.getTmsetMinY()));
        r.push_back(column(data.getTmsetMaxX()));
        r.push_back(column(data.getTmsetMaxY()));

        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }

    {
        const tilercommon::TileMatrixMath tmm(data.getTmsetMinX(), data.getTmsetMinY(),
            data.getTmsetMaxX(), data.getTmsetMaxY(),
            data.getNumColsAtL0(), data.getNumRowsAtL0());

        for (uint32_t level=0; level<=data.getMaxLevel(); ++level)
        {
            const std::string sql =
                "INSERT INTO gpkg_pctile_matrix"
                " (table_name, zoom_level, matrix_width, matrix_height)"
                " VALUES (?, ?, ?, ?)";

            records rs;
            row r;

            uint32_t numCols, numRows;
            numCols = tmm.numColsAtLevel(level);
            numRows = tmm.numRowsAtLevel(level);

            r.push_back(column(data.getName()));
            r.push_back(column(level));
            r.push_back(column(numCols));
            r.push_back(column(numRows));
            rs.push_back(r);

            m_sqlite->insert(sql, rs);
        }
    }

    writeDimensions(data);

    writeMetadata(data);

    e_tileTablesWritten.stop();
}


void GeoPackageWriter::writeMetadata(const GpkgMatrixSet& data)
{
    uint32_t metadata_id;

    {
        const std::string sql =
            "INSERT INTO gpkg_metadata"
            " (md_scope, md_standard_uri, mime_type, metadata)"
            " VALUES (?, ?, ?, ?)";
            
        records rs;
        row r;

        r.push_back(column("dataset"));
        r.push_back(column("LAS"));
        r.push_back(column("text/plain"));
        r.push_back(column(data.getLasMetadata()));

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
        r.push_back(column(data.getName())); // table_name
        r.push_back(column("table")); // column_name
        r.push_back(column("NULL")); // row_id_value
        r.push_back(column(data.getDateTime())); // timestamp
        r.push_back(column(metadata_id)); // md_file_id
        r.push_back(column("NULL")); // md_parent_id

        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }
}


void GeoPackageWriter::writeDimensions(const GpkgMatrixSet& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    const std::string sql =
        "INSERT INTO gpkg_pctile_dimension_set"
        " (table_name, dimension_name, data_type, ordinal_position, description,"
        " minimum, mean, maximum)"
        " VALUES (?, ?, ?, ?, ?, ?, ?, ?)";

    int i = 0;
    for (auto dim: data.getDimensions())
    {
        records rs;
        row r;

        r.push_back(column(data.getName())); // table_name
        r.push_back(column(dim.getName()));
        r.push_back(column(dim.getDataType()));
        r.push_back(column(i)); // ordinal_position
        r.push_back(column("...description..."));
        r.push_back(column(dim.getMinimum()));
        r.push_back(column(dim.getMean()));
        r.push_back(column(dim.getMaximum()));

        rs.push_back(r);

        m_sqlite->insert(sql, rs);

        ++i;
    }
}


void GeoPackageWriter::writeTile(const std::string& tileTableName, const GpkgTile& data)
{
    if (!m_sqlite)
    {
        throw pdal_error("RialtoDB: invalid state (session does exist)");
    }

    e_tilesWritten.start();

    const uint32_t buflen = data.getPatch().size();
    const unsigned char* buf = data.getPatch().getPointer();
    assert(buf);
    assert(buflen);

    {
        const std::string sql =
            "INSERT INTO " + tileTableName +
            " (zoom_level, tile_column, tile_row, tile_data, num_points, child_mask)"
            " VALUES (?, ?, ?, ?, ?, ?)";

        records rs;
        row r;

        r.push_back(column(data.getLevel()));
        r.push_back(column(data.getColumn()));
        r.push_back(column(data.getRow()));
        r.push_back(blob((char*)buf, (size_t)buflen));
        r.push_back(column(data.getNumPoints()));
        r.push_back(column(data.getMask()));
        rs.push_back(r);

        m_sqlite->insert(sql, rs);
    }

    e_tilesWritten.stop();

    m_numPointsWritten += data.getNumPoints();
}


void GeoPackageWriter::childDumpStats() const
{
    std::cout << "GeoPackageWriter stats" << std::endl;

    e_tilesWritten.dump();
    e_tileTablesWritten.dump();

    std::cout << "    pointsWritten: ";

    if (m_numPointsWritten)
    {
        std::cout << m_numPointsWritten;
    }
    else
    {
        std::cout << "-";
    }

    std::cout << std::endl;
}


} // namespace rialto
} // namespace pdal
