@echo off

cmake -G "Visual Studio 11 2012 Win64" ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DBUILD_PLUGIN_ATTRIBUTE=%PDAL_OPTIONAL_COMPONENTS% ^
    -DBUILD_PLUGIN_CPD=OFF ^
    -DBUILD_PLUGIN_GREYHOUND=OFF ^
    -DBUILD_PLUGIN_HEXBIN=OFF ^
    -DBUILD_PLUGIN_ICEBRIDGE=OFF ^
    -DBUILD_PLUGIN_MRSID=OFF ^
    -DBUILD_PLUGIN_NITF=OFF ^
    -DBUILD_PLUGIN_OCI=OFF ^
    -DBUILD_PLUGIN_P2G=OFF ^
    -DBUILD_PLUGIN_PCL=OFF ^
    -DBUILD_PLUGIN_PGPOINTCLOUD=OFF ^
    -DBUILD_PLUGIN_SQLITE=OFF ^
    -DBUILD_PLUGIN_RIVLIB=OFF ^
    -DBUILD_PLUGIN_PYTHON=%PDAL_OPTIONAL_COMPONENTS% ^
    -DENABLE_CTEST=OFF ^
    -DWITH_APPS=ON ^
    -DWITH_LAZPERF=%PDAL_OPTIONAL_COMPONENTS% ^
    -DWITH_GEOTIFF=%PDAL_OPTIONAL_COMPONENTS% ^
    -DWITH_ICONV=%PDAL_OPTIONAL_COMPONENTS% ^
    -DWITH_LASZIP=%PDAL_OPTIONAL_COMPONENTS% ^
    -DWITH_LIBXML2=OFF ^
    -DWITH_TESTS=ON ^
    -Dgtest_force_shared_crt=ON ^
    .
