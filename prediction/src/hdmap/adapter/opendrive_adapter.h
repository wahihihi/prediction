//
// Created by ai on 23-3-7.
//

#ifndef PREDICTION_OPENDRIVE_ADAPTER_H
#define PREDICTION_OPENDRIVE_ADAPTER_H
#include "glog/logging.h"
#include "prediction/common_include.h"
#include "hdmap/parser/common_define.h"
#include "hdmap/parser/header_xml_parser.h"
#include "hdmap/parser/road_xml_parser.h"
#include "hdmap/entity/map.h"
#include "hdmap/entity/map_road.h"
#include "hdmap/entity/map_lane.h"
#include "tinyxml2.h"
namespace aptiv {
namespace hdmap {
namespace adapter {

class OpendriveAdapter{
public:
    static bool LoadData(const std::string& filename,std::shared_ptr<aptiv::hdmap::entity::Map> map);
};

}
}
}


#endif //PREDICTION_OPENDRIVE_ADAPTER_H
