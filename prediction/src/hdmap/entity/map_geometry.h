//
// Created by swh on 23-3-6.
//

#ifndef PREDICTION_MAP_GEOMETRY_H
#define PREDICTION_MAP_GEOMETRY_H
#include "prediction/common_include.h"

struct PointENU{
    double x;
    double y;
    double z;
};

struct LineSegment{
    std::vector<PointENU> points;
};

struct CurveSegment{
    enum CurveType{
        CurveType_None,
        CurveType_Line,
        CurveType_ARC
    };
    CurveType curveType = CurveType_None;
    LineSegment lineSegment;


    double s;
    PointENU start_position;
    double heading;
    double length;
};

struct Curve{
    std::vector<CurveSegment> segment;
};
#endif //PREDICTION_MAP_GEOMETRY_H
