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
using Header = aptiv::hdmap::entity::Header;
using Road = aptiv::hdmap::entity::Road;
using RoadSection = aptiv::hdmap::entity::RoadSection;
using Lane = aptiv::hdmap::entity::Lane;


struct LaneInternal {
    Lane lane;
};

struct RoadSectionInternal {
    std::string id;
    RoadSection section;
    std::vector<LaneInternal> lanes;
};

struct RoadInternal {
    std::string id;
    Road road;

    bool in_junction;
    std::string junction_id;
    std::string type;
    std::vector<RoadSectionInternal> sections;

    RoadInternal() : in_junction(false) { junction_id = ""; }
};

}
}
}
#endif //PREDICTION_COMMON_DEFINE_H
