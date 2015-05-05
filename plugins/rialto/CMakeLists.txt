#
# Rialto plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)

set(file_srcs
    io/RialtoWriter.cpp
    io/RialtoFileWriter.cpp
    )
    
set(file_incs
    io/RialtoWriter.hpp
    io/RialtoFileWriter.hpp
    )

set(db_srcs
    io/RialtoWriter.cpp
    io/RialtoDbWriter.cpp
    io/RialtoDb.cpp
    )

set(db_incs
    io/RialtoWriter.hpp
    io/RialtoDbWriter.hpp
    io/RialtoDb.hpp
)

PDAL_ADD_PLUGIN(dbwriter_libname writer rialtodb
    FILES "${db_srcs}" "${db_incs}"
    LINK_WITH ${SQLITE3_LIBRARY})

PDAL_ADD_PLUGIN(filewriter_libname writer rialtofile
    FILES "${file_srcs}" "${file_incs}")

#
# Rialto tests
#
if(BUILD_RIALTO_TESTS)
	PDAL_ADD_TEST(rialtofiletest
        FILES test/RialtoFileWriterTest.cpp
        LINK_WITH ${filewriter_libname}
    )
    PDAL_ADD_TEST(rialtodbtest
        FILES test/RialtoDbWriterTest.cpp
        LINK_WITH ${dbwriter_libname}
    )
endif()
