//
// Created by swh on 23-3-6.
//

#ifndef PREDICTION_MAP_ROAD_H
#define PREDICTION_MAP_ROAD_H
#pragma once

#include "prediction/common_include.h"
#include "hdmap/entity/map_geometry.h"

namespace aptiv{
namespace hdmap{
namespace entity{

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

struct Successor {
    std::string elementType;
    int elementId;
    std::string contactPoint;
};

struct Predecesor {
    std::string elementType;
    int elementId;
    std::string contactPoint;
};

struct Link {
    Predecesor predecesor;
    Successor successor;
};

struct Road {
    std::string id;
    std::string name;
    double length;
    std::vector<RoadSection> section;
    std::string junctionId;
    double speed_max;

    enum RoadType{
        RoadType_UNKNOWN,
        RoadType_HIGHWAY,
        RoadType_TOWN,
        RoadType_PARK
    };
    RoadType type = RoadType::RoadType_UNKNOWN;

    Link link;
};
}
}
}
#endif //PREDICTION_MAP_ROAD_H
