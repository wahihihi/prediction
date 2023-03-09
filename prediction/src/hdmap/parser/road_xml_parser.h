//
// Created by ai on 23-3-7.
//

#ifndef PREDICTION_ROAD_XML_PARSER_H
#define PREDICTION_ROAD_XML_PARSER_H
#include "prediction/common_include.h"
#include "hdmap/parser/common_define.h"
#include "hdmap/parser/xml_parse_util.h"
#include "hdmap/parser/lane_xml_parser.h"

#include "hdmap/entity/map_road.h"

namespace aptiv{
namespace hdmap{
namespace parser{

class RoadXmlParser {
public:
    static int Parse(const tinyxml2::XMLElement& node,RoadInternal* roads);
private:
    static void Parse_road_objects(const tinyxml2::XMLElement& node,RoadInternal* road_info );
    static int to_road_type(std::string type,Road::RoadType roadType);
};


}
}
}


#endif //PREDICTION_ROAD_XML_PARSER_H
