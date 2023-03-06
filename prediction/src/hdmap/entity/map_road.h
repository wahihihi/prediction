//
// Created by swh on 23-3-6.
//

#ifndef PREDICTION_MAP_ROAD_H
#define PREDICTION_MAP_ROAD_H
#pragma once

#include "prediction/common_include.h"
#include "hdmap/entity/map_geometry.h"

struct Projection {
    std::string proj;
};

struct Header{
    std::string version ;
    std::string date;
    std::shared_ptr<Projection> projection;
    size_t revMajor;
    size_t revMinor;
    std::string name;
    double north = 0.0;
    double south = 0.0;
    double east = 0.0;
    double west = 0.0;
    std::string vendor;
};



struct BoundaryEdge{
    Curve curve;
    enum BoundaryEdge_Type{
        BoundaryEdge_UNKNOWN,
        BoundaryEdge_NORMAL,
        BoundaryEdge_LEFT,
        BoundaryEdge_RIGHT
    };
    BoundaryEdge_Type type = BoundaryEdge::BoundaryEdge_UNKNOWN;
};

struct BoundaryPolygon{
    std::vector<BoundaryEdge> edge;
};

struct RoadBoundary{
    BoundaryPolygon outer_polygon;
};

struct RoadSection{
    std::string id;
    std::vector<std::string> lane_ids;
    RoadBoundary boundary;
};

struct SpeedLimit{
    std::string max;
    std::string unit;
};

struct Road {
    std::string id;
    std::vector<RoadSection> section;
    std::string junctionId;

    enum RoadType{
        RoadType_UNKNOWN,
        RoadType_HIGHWAY,
        RoadType_TOWN,
        RoadType_PARK
    };
    RoadType type = RoadType::RoadType_UNKNOWN;

    SpeedLimit speed;
};
#endif //PREDICTION_MAP_ROAD_H
