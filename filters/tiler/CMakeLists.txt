#
# Tiler filter CMake configuration
#

#
# Tiler Filter
#
set(srcs
    TilerCommon.cpp
    TilerFilter.cpp
)

set(incs
    TilerCommon.hpp
    TilerFilter.hpp
)

PDAL_ADD_DRIVER(filter tiler "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
