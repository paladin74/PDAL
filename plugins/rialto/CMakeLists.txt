#
# Rialto plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)

set(srcsF
    io/RialtoWriter.cpp
    io/RialtoFileWriter.cpp)
    
set(incsF
    io/RialtoWriter.hpp
    io/RialtoFileWriter.hpp
)

set(srcsD
    io/RialtoWriter.cpp
    io/RialtoDbWriter.cpp)

set(incsD
    io/RialtoWriter.hpp
    io/RialtoDbWriter.hpp
)

PDAL_ADD_PLUGIN(dbwriter_libname writer rialtodb
    FILES "${srcsD}" "${incsD}"
    LINK_WITH ${SQLITE3_LIBRARY})

PDAL_ADD_PLUGIN(filewriter_libname writer rialtofile
    FILES "${srcsF}" "${incsF}")

#
# Rialto tests
#
if(BUILD_RIALTO_TESTS)
	PDAL_ADD_TEST(rialtofiletest
        FILES test/RialtoFileWriterTest.cpp
        LINK_WITH ${filewriter_libname}
    )
    PDAL_ADD_TEST(rialtodbtest
        FILES test/RialtoDb.cpp test/RialtoDbTest.cpp test/RialtoDbWriterTest.cpp
        LINK_WITH ${dbwriter_libname}
    )
endif()
