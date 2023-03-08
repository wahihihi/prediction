//
// Created by swh on 23-3-6.
//

#ifndef PREDICTION_MAP_LANE_H
#define PREDICTION_MAP_LANE_H
#include "prediction/common_include.h"
#include "hdmap/entity/map_geometry.h"
namespace aptiv{
namespace hdmap{
namespace entity{

struct LaneBoundary{
    Curve curve;
    double length;
    bool virtual_;

};

struct LaneSampleAssociation{
    double s;
    double width;
};

struct LaneWidth{
    double sOffset=0.0;
    double a=0.0;
    double b=0.0;
    double c=0.0;
    double d=0.0;

};

struct LaneOffset{
    double s;
    double a;
    double b;
    double c;
    double d;
};

struct Lane{
    std::string id;
    Curve central_curve;
    LaneBoundary left_boundary;
    LaneBoundary right_boundary;

    double length;
    std::string predecessor_id;
    std::string successor_id;

    std::vector<LaneWidth> lane_widths;
    std::vector<std::string> left_neighbor_forward_lane_id;
    std::vector<std::string> right_neighbor_forward_lane_id;

    enum LaneType{
        LaneType_None,
        LaneType_Driving,
        LaneType_Sidwalk,
        LaneType_Shoulder,
        LaneType_Parking,
        LaneType_Biking
    };
    LaneType laneType = LaneType::LaneType_None;

    std::vector<std::string> left_neighbor_reverse_lane_id;
    std::vector<std::string> right_neighbor_reverse_lane_id;

    std::string junction_id;

    std::vector<LaneSampleAssociation> left_lane_sample;
    std::vector<LaneSampleAssociation> right_lane_sample;

    enum LaneDirection{
        FORWARD,
        BACKWARD,
        BIDIRECTION
    };
    LaneDirection laneDirection = LaneDirection::FORWARD;


    std::vector<LaneSampleAssociation> left_road_sample;
    std::vector<LaneSampleAssociation> right_road_sample;

    std::vector<std::string > self_reverse_lane_id;
};
}
}
}

#endif //PREDICTION_MAP_LANE_H
