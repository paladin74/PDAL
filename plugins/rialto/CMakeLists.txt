#
# Rialto plugin CMake configuration
#

include (${PDAL_CMAKE_DIR}/sqlite.cmake)

set(file_writer_srcs
    io/RialtoWriter.cpp
    io/RialtoFileWriter.cpp
    )

set(file_writer_incs
    io/RialtoWriter.hpp
    io/RialtoFileWriter.hpp
    )

set(db_writer_srcs
    io/RialtoWriter.cpp
    io/RialtoDbWriter.cpp
    io/RialtoDb.cpp
    )

set(db_writer_incs
    io/RialtoWriter.hpp
    io/RialtoDbWriter.hpp
    io/RialtoDb.hpp
)

set(file_reader_srcs
    io/RialtoReader.cpp
    io/RialtoFileReader.cpp
    )

set(file_reader_incs
    io/RialtoReader.hpp
    io/RialtoFileReader.hpp
    )

set(db_reader_srcs
    io/RialtoReader.cpp
    io/RialtoDbReader.cpp
    io/RialtoDb.cpp
    )

set(db_reader_incs
    io/RialtoReader.hpp
    io/RialtoDbReader.hpp
    io/RialtoDb.hpp
)

PDAL_ADD_PLUGIN(dbwriter_libname writer rialtodbwriter
    FILES "${db_writer_srcs}" "${db_writer_incs}"
    LINK_WITH ${SQLITE3_LIBRARY})

PDAL_ADD_PLUGIN(dbreader_libname reader rialtodbreader
    FILES "${db_reader_srcs}" "${db_reader_incs}"
    LINK_WITH ${SQLITE3_LIBRARY})

PDAL_ADD_PLUGIN(filewriter_libname writer rialtofilewriter
    FILES "${file_writer_srcs}" "${file_writer_incs}")

PDAL_ADD_PLUGIN(filereader_libname reader rialtofilereader
    FILES "${file_reader_srcs}" "${file_reader_incs}")

#
# Rialto tests
#
if(BUILD_RIALTO_TESTS)
	  PDAL_ADD_TEST(rialtofilewritertest
        FILES test/RialtoFileWriterTest.cpp test/RialtoTest.cpp
        LINK_WITH ${filewriter_libname}
    )
    PDAL_ADD_TEST(rialtodbwritertest
        FILES test/RialtoDbWriterTest.cpp test/RialtoTest.cpp
        LINK_WITH ${dbwriter_libname}
    )
    PDAL_ADD_TEST(rialtofilereadertest
        FILES test/RialtoFileReaderTest.cpp test/RialtoTest.cpp
        LINK_WITH ${filereader_libname}
    )
    PDAL_ADD_TEST(rialtodbreadertest
        FILES test/RialtoDbReaderTest.cpp test/RialtoTest.cpp
        LINK_WITH ${dbreader_libname}
    )
endif()
