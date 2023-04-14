//
// Created by ai on 23-4-14.
//

#ifndef PREDICTION_2_SRC_ADAPTER_OPENDRIVE_ADAPTER_H_
#define PREDICTION_2_SRC_ADAPTER_OPENDRIVE_ADAPTER_H_
#include <string>
#include "prediction_2/geometry/element.h"
#include "prediction_2/entity/entity_map.h"
#include "prediction_2/geometry/element.h"
namespace prediction {
namespace adapter {
using namespace prediction::entity;

class opendrive_adapter {
 public:
  static bool LoadData(const std::string& filename,Entity_Map* entity_map);
};

}
}

#endif //PREDICTION_2_SRC_ADAPTER_OPENDRIVE_ADAPTER_H_
