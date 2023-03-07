//
// Created by ai on 23-3-7.
//

#ifndef PREDICTION_MAP_H
#define PREDICTION_MAP_H
#include "prediction/common_include.h"
#include "hdmap/entity/map_road.h"

namespace aptiv{
namespace hdmap{
namespace entity{

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

struct Map{
    std::shared_ptr<Header> header;
    std::vector<Road> roads;
};

}
}
}
#endif //PREDICTION_MAP_H
