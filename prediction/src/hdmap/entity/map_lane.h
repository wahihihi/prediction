//
// Created by swh on 23-3-6.
//

#ifndef PREDICTION_MAP_LANE_H
#define PREDICTION_MAP_LANE_H
#include "prediction/common_include.h"
#include "hdmap/entity/map_geometry.h"

struct LaneBoundary{
    Curve curve;
    double length;
    bool virtual_;

};

struct Lane{
    std::string id;
    Curve central_curve;
    LaneBoundary left_boundary;
    LaneBoundary right_boundary;

    double length;
    std::string predecessor_id;
    std::string successor_id;


};

#endif //PREDICTION_MAP_LANE_H
