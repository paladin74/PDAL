

PDAL_INC=/Users/mgerlek/work/dev/PDAL/include
PDAL_LIB=/Users/mgerlek/work/dev/PDAL/lib

all:
	g++ -o rialto rialto.cpp Tool.cpp \
	-g -std=c++11 -ferror-limit=3 \
	-isystem /usr/local/include \
	-isystem /usr/include/libxml2 \
	-I $(PDAL_INC) -I /usr/local/include \
	-L$(PDAL_LIB) \
	-lpdalcpp -lpdal_util -llaszip \
	-lpdal_plugin_writer_rialtofile -lpdal_plugin_writer_rialtodb -lpdal_plugin_reader_rialtodb
