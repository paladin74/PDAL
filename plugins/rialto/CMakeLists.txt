#
# Rialto plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)

set(file_writer_srcs
    io/RialtoSupport.hpp
    io/RialtoSupport.cpp
    io/RialtoWriter.hpp
    io/RialtoWriter.cpp
    io/RialtoFileWriter.hpp
    io/RialtoFileWriter.cpp
    )

set(db_writer_srcs
    io/RialtoSupport.hpp
    io/RialtoSupport.cpp
    io/RialtoDb.hpp
    io/RialtoDb.cpp
    io/RialtoWriter.hpp
    io/RialtoWriter.cpp
    io/RialtoDbWriter.hpp
    io/RialtoDbWriter.cpp
)

set(db_reader_srcs
    io/RialtoSupport.hpp
    io/RialtoSupport.cpp
    io/RialtoDb.hpp
    io/RialtoDb.cpp
    io/RialtoDbReader.hpp
    io/RialtoDbReader.cpp
)

PDAL_ADD_PLUGIN(dbwriter_libname writer rialtodb
    FILES "${db_writer_srcs}"
    LINK_WITH ${SQLITE3_LIBRARY})

PDAL_ADD_PLUGIN(dbreader_libname reader rialtodb
    FILES "${db_reader_srcs}"
    LINK_WITH ${SQLITE3_LIBRARY})

PDAL_ADD_PLUGIN(filewriter_libname writer rialtofile
    FILES "${file_writer_srcs}"
    )

#
# Rialto tests
#
if(BUILD_RIALTO_TESTS)
	  PDAL_ADD_TEST(rialtofilewritertest
        FILES test/RialtoFileWriterTest.cpp
        LINK_WITH ${filewriter_libname}
    )
    PDAL_ADD_TEST(rialtodbwritertest
        FILES test/RialtoDbWriterTest.cpp
        LINK_WITH ${dbwriter_libname} ${dbreader_libname}
    )
    PDAL_ADD_TEST(rialtodbreadertest
        FILES test/RialtoDbReaderTest.cpp
        LINK_WITH ${dbreader_libname} 
    )
endif()
