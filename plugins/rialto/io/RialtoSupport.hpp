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

#pragma once

#include <pdal/pdal.hpp>

namespace pdal
{
namespace rialto
{


class MyPatch
{
public:
    uint32_t size() const;
    void clear();
    bool isEmpty() const;
    const std::vector<unsigned char>& getVector() const;
    const unsigned char* getPointer() const;
    void importFromVector(const std::vector<uint8_t>& vec);
    void importFromPV(const PointView& view);

    // does an append to the PV (does not start at index 0)
    void exportToPV(size_t numPoints, PointViewPtr view) const;

private:
    std::vector<uint8_t> m_vector;
};


class DimensionInfo
{
public:
    void set(const std::string& name,
             uint32_t position,
             const std::string& dataType,
             const std::string& description,
             double minimum,
             double mean,
             double maximum);

    static void import(MetadataNode tileSetNode,
                PointLayoutPtr layout,
                std::vector<DimensionInfo>& infoList);

    const std::string& getName() const { return m_name; }
    uint32_t getPosition() const { return m_position; }
    const std::string& getDataType() const { return m_dataType; };
    const std::string& getDescription() const { return m_description; }
    double getMinimum() const { return m_minimum; }
    double getMean() const { return m_mean; }
    double getMaximum() const { return m_maximum; }
    
private:
    std::string m_name;
    uint32_t m_position;
    std::string m_dataType;
    std::string m_description;
    double m_minimum;
    double m_mean;
    double m_maximum;
};


// Rialto has some hard-coded restrictions:
//   we always use EPSG:4326
//   we always start with two tiles at the root
//   we always cover the whole globe at the root
//   we always do power-of-two reductions
//   we store all levels between 0 and max, inclusive
class TileSetInfo
{
public:
    TileSetInfo() {}

    TileSetInfo(const std::string& tileSetName,
                MetadataNode tileSetNode,
                PointLayoutPtr layout,
                const std::string& datetime);

    void set(const std::string& datetime,
             const std::string& name,
             uint32_t maxLevel,
             uint32_t numDimensions,
             double data_min_x,
             double data_min_y,
             double data_max_x,
             double data_max_y,
             double tmset_min_x,
             double tmset_min_y,
             double tmset_max_x,
             double tmset_max_y);

    std::string getDateTime() const { return m_datetime; }
    std::string getName() const { return m_name; } // aka filename
    uint32_t getMaxLevel() const { return m_maxLevel; }
    uint32_t getNumDimensions() const { return m_numDimensions; }
    const std::vector<DimensionInfo>& getDimensions() const { return m_dimensions; };
    std::vector<DimensionInfo>& getDimensionsRef() { return m_dimensions; };

    double getDataMinX() const { return m_data_min_x; } // data extents
    double getDataMinY() const { return m_data_min_y; }
    double getDataMaxX() const { return m_data_max_x; }
    double getDataMaxY() const { return m_data_max_y; }

    double getTmsetMinX() const { return m_tmset_min_x; }
    double getTmsetMinY() const { return m_tmset_min_y; }
    double getTmsetMaxX() const { return m_tmset_max_x; }
    double getTmsetMaxY() const { return m_tmset_max_y; }

private:
    std::string m_datetime;
    std::string m_name; // aka filename
    uint32_t m_maxLevel;
    uint32_t m_numDimensions;
    std::vector<DimensionInfo> m_dimensions;
    double m_data_min_x; // data extents
    double m_data_min_y;
    double m_data_max_x;
    double m_data_max_y;
    double m_tmset_min_x; // tile extents
    double m_tmset_min_y;
    double m_tmset_max_x;
    double m_tmset_max_y;
};


class TileInfo
{
public:
    TileInfo() {}

    TileInfo(PointView* view, uint32_t level, uint32_t column, uint32_t row, uint32_t mask);

    void set(uint32_t level,
             uint32_t column,
             uint32_t row,
             uint32_t numPoints,
             uint32_t mask);

    uint32_t getLevel() const { return m_level; }
    uint32_t getColumn() const { return m_column; }
    uint32_t getRow() const { return m_row; }
    uint32_t getNumPoints() const { return m_numPoints; }
    uint32_t getMask() const { return m_mask; }
    const MyPatch& getPatch() const { return m_patch; }
    MyPatch& getPatchRef() { return m_patch; }

private:
    uint32_t m_level;
    uint32_t m_column;
    uint32_t m_row;
    uint32_t m_numPoints;
    uint32_t m_mask;
    MyPatch m_patch;
};


class WriterAssister
{
public:
    void write(const PointViewPtr viewPtr);
    void ready(PointTableRef table);
    void writeEmptyTiles();

    std::string m_tileSetName;

protected:
    virtual void writeHeader(const std::string& tileSetName,
                             MetadataNode tileSetNode,
                             PointLayoutPtr layout,
                             const std::string& datetime)=0;
    virtual void writeTile(const std::string& tileSetName, PointView*,
                           uint32_t level, uint32_t col, uint32_t row, uint32_t mask)=0;

private:
    std::map<uint32_t, uint32_t> m_pointViewMap2; // PV id to array index
    uint32_t* m_tileMetadata;
    uint32_t m_numTiles;

    MetadataNode tileSetNode;
    void makePointViewMap();
};


class RialtoEvent
{
public:
    // Event e_foo("foo");
    // e.start();
    // ...work...
    // e.stop();
    // e.dump();
    RialtoEvent(const std::string& name);
    ~RialtoEvent();

    void start();
    void stop();

    void dump() const;

    // clock_t start = timerStart();
    // <spin cycles>
    // uint32_t millis = timerStop(start);
    static clock_t timerStart();
    static double timerStop(clock_t start);

private:
     const std::string m_name;
     uint32_t m_count;
     double m_millis;
     clock_t m_start;
};


} // namespace rialto
} // namespace pdal
