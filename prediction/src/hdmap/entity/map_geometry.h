//
// Created by swh on 23-3-6.
//

#ifndef PREDICTION_MAP_GEOMETRY_H
#define PREDICTION_MAP_GEOMETRY_H
#include "prediction/common_include.h"
namespace aptiv{
namespace hdmap{
namespace entity{
struct PointENU{
    PointENU() = default;
    PointENU(double _x, double _y, double _z,double _s, double _hdg):x(_x),y(_y),z(_z),s(_s),hdg(_hdg){}
    double x;
    double y;
    double z;
    double s;
    double hdg;
    double curveture = 0.0;
};

struct LineSegment{
    std::vector<PointENU> points;
};

struct CurveSegment{
    CurveSegment() = default;
    enum CurveType{
        CurveType_None,
        CurveType_Line,
        CurveType_ARC
    };
    CurveType curveType = CurveType_None;
    LineSegment lineSegment;


    double s;
    double laneOffset;
    PointENU start_position;
    double heading;
    double length;
};

struct Curve{
    std::vector<CurveSegment> segment;
};
}
}
}
#endif //PREDICTION_MAP_GEOMETRY_H
