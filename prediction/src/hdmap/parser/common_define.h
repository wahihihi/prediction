//
// Created by ai on 23-3-7.
//

#ifndef PREDICTION_COMMON_DEFINE_H
#define PREDICTION_COMMON_DEFINE_H
#include "hdmap/entity/map.h"
#include "hdmap/entity/map_road.h"
#include "hdmap/entity/map_lane.h"

namespace aptiv {
namespace hdmap {
namespace parser {
using MpHeader = aptiv::hdmap::entity::Header;
using MpRoad = aptiv::hdmap::entity::Road;
using MpRoadSection = aptiv::hdmap::entity::RoadSection;
using MpLane = aptiv::hdmap::entity::Lane;
using MpLaneOffSet = aptiv::hdmap::entity::LaneOffset;


struct LaneInternal {
    MpLane lane;
    bool isCenter = false;
    bool isMostLeft = false;
    bool isMostRight = false;
};

struct RoadSectionInternal {
    std::string id;
    MpRoadSection section;
    std::vector<LaneInternal> lanes;
};

struct RoadInternal {
    std::string id;
    MpRoad road;

    bool in_junction;
    std::string junction_id;
    std::string type;
    std::vector<MpLaneOffSet> laneOffsetSet;
    std::vector<RoadSectionInternal> sections;

    RoadInternal() : in_junction(false) { junction_id = ""; }
};

}
}
}
#endif //PREDICTION_COMMON_DEFINE_H
