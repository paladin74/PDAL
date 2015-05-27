#
# Rialto plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)

set(common_srcs
    io/Event.hpp
    io/GeoPackage.hpp
    io/GeoPackage.cpp
    io/GeoPackageCommon.hpp
    io/GeoPackageCommon.cpp
    io/GeoPackageManager.hpp
    io/GeoPackageManager.cpp
    io/GeoPackageReader.hpp
    io/GeoPackageReader.cpp
    io/GeoPackageWriter.hpp
    io/GeoPackageWriter.cpp
    io/RialtoWriterAssister.hpp
    io/RialtoWriterAssister.cpp
    )

set(file_writer_srcs
    io/RialtoFileWriter.hpp
    io/RialtoFileWriter.cpp
    )

set(db_writer_srcs
    io/RialtoDbWriter.hpp
    io/RialtoDbWriter.cpp
)

set(db_reader_srcs
    io/RialtoDbReader.hpp
    io/RialtoDbReader.cpp
)

PDAL_ADD_PLUGIN(dbwriter_libname writer rialtodb
    FILES "${db_writer_srcs}" "${common_srcs}"
    LINK_WITH ${SQLITE3_LIBRARY}
    )

PDAL_ADD_PLUGIN(dbreader_libname reader rialtodb
    FILES "${db_reader_srcs}" "${common_srcs}"
    LINK_WITH ${SQLITE3_LIBRARY}
    )

PDAL_ADD_PLUGIN(filewriter_libname writer rialtofile
    FILES "${file_writer_srcs}" "${common_srcs}"
    LINK_WITH ${SQLITE3_LIBRARY}
    )

#
# Rialto tests
#
if(BUILD_RIALTO_TESTS)
    PDAL_ADD_TEST(geopackagetest
        FILES test/GeoPackageTest.cpp
        LINK_WITH ${dbwriter_libname} ${dbreader_libname}
    )
    PDAL_ADD_TEST(rialtofilewritertest
        FILES test/RialtoFileWriterTest.cpp
        LINK_WITH ${filewriter_libname} ${dbwriter_libname} ${dbreader_libname}
    )
    PDAL_ADD_TEST(rialtodbwritertest
        FILES test/RialtoDbWriterTest.cpp
        LINK_WITH ${dbwriter_libname} ${dbreader_libname}
    )
    PDAL_ADD_TEST(rialtodbreadertest
        FILES test/RialtoDbReaderTest.cpp
        LINK_WITH ${dbreader_libname} ${dbwriter_libname}
    )
endif()
