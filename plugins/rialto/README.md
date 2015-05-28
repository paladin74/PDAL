Rialto/GeoPackage Library
=========================

This library implements a point cloud extension to the GeoPackage database
format, in support of the Rialto project.



The Main Classes, and what they do
----------------------------------

* GeoPackage classes

  * GeoPackage
    * abstract base class for direct/low-level geopackage operations
    * hides all uses of sqlite calls

  * GeoPackageManager
    * supports DB creation, getting info about matrix sets, deleting matrix sets

  * GeoPackageReader
    * supports reading a matrix set from the DB
    * these are the low-level operations, not a pdal::Reader class

  * GeoPackageWriter
    * supports writing a matrix set to an existing DB
    * these are the low-level operations, not a pdal::Writer class

  * GpkgDimension
    * container for the "dimension" info of a tile matrix set
    * essentially mirrors the database table schema
  
  * GpkgMatrixSet
    * container for basic info about a tile matrix set
    * essentially mirrors the database table schema
 
  * GpkgTile
    * container for a tile's data
    * essentially mirrors the database table schema

* Rialto "Stage" classes

  * RialtoDBReader
    * a pdal::Reader class which allows reading tile data from a single tile
      matrix set of a geopackage database

  * RialtoDBWriter
    * a pdal::Writer class which allows writing tile data to a single tile matrix
      set of an existing geopackage database

  * RialtoFileWriter
    * a pdal::Writer which allows writing a single file to a tile matrix set on
      disk, suitable for use with the Cesium/Rialto viewer

  * RialtoFileReader
    * _there is no RialtoFileReader class. although there might be some day_
    * _the only Rialto file reader implementation lives inside the Ceisum/Rialto
      viewer, written in JavaScript/Dart_

    * RialtoWriterAssister - helper class for the two Rialto writers _(should be
      a base class they both derive from)_


Notes on Reader and Writer behaviors
------------------------------------

* RialtoDBReader
  * You may call execute() multiple times
    * but you need to remove and re-add the bbox option each time
   * Options:
    * filename: required
    * name: name of matrix set (optional, defaults to 1st one in database but throws if db has more than one)
    * bbox: query bounds (optional, defaults to matrix set bounds)
    * level: zoom level to use (optional, defaults to max level)
* RialtoDbWriter
  * Options
    * filename - name of database (required)
    * name - name of table (required), also used as "human-readable identifier"
    * numRows, numCols - grid size at level 0 (required)
    * description - required
    * timestamp - date of "last change" in ISO 8601 format (required)
      

OGC GeoPackage Nomenclature
-----------------------

* "Tile"
  * a set of points, which is a subset of the entire "file"
  * bounded by a given bbox, with (0,0) at lower-left

* "Tile Matrix"
  * a set of tiles, making up one zoom level
  * a tile matrix is arranged as a 2D grid, C columns by R rows
  * tile (0,0) is upper-left

* "Tile Matrix Set"
  * a set of tile matrices, covering all the levels of the hierarchy



Other Notes
-----------

* The file extension to use is ".gpkg".


TODO
-----
* store the actual metadata
* writer should overwrite the existing table (or fail)
* order-dependent in test case RialtoDbWriterTest
* make table_name be a unique id, add separate human-readable id
