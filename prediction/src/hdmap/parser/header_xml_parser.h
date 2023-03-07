//
// Created by ai on 23-3-1.
//

#ifndef PREDICTION_1_0_HEADERXMLPARSER_H
#define PREDICTION_1_0_HEADERXMLPARSER_H
#include <memory>

#include "tinyxml2.h"
#include "hdmap/entity/map.h"

namespace aptiv{
namespace hdmap{
namespace adapter{

class HeaderXMlParser{
public:
    static int Parse(const tinyxml2::XMLElement& node,std::shared_ptr<aptiv::hdmap::entity::Header> header);
};

}
}
}


#endif //PREDICTION_1_0_HEADERXMLPARSER_H
