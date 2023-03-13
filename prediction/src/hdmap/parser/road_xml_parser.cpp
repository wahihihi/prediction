//
// Created by ai on 23-3-7.
//

#include "road_xml_parser.h"
#include "iostream"

namespace aptiv{
namespace hdmap{
namespace parser{
    using namespace aptiv::hdmap::entity;
int RoadXmlParser::Parse(const tinyxml2::XMLElement &node, RoadInternal* roads) {
    CHECK_NOTNULL(roads);

    auto road_node = node.FirstChildElement("road");
    while (road_node){

        RoadInternal roadInternal;
        std::string id_str;
        std::string name_str;
        std::string length_str;
        std::string junction_id_str;

        XmlParserUtil::QueryStringAttribute(*road_node,"id",&id_str);
        XmlParserUtil::QueryStringAttribute(*road_node,"name",&name_str);
        XmlParserUtil::QueryStringAttribute(*road_node,"length",&length_str);
        XmlParserUtil::QueryStringAttribute(*road_node,"junction",&junction_id_str);


        roadInternal.id = id_str;
        roadInternal.road.id = id_str;
        roadInternal.road.name = name_str;
        roadInternal.road.length = std::stod(length_str);
        roadInternal.road.junctionId = junction_id_str;
        roadInternal.junction_id = junction_id_str;

        int checker = 0;
        // linke tag
        const tinyxml2::XMLElement* link_node = road_node->FirstChildElement("link");
        if (link_node){
            Predecesor predecesor;
            const tinyxml2::XMLElement* predecessor_node = link_node->FirstChildElement("predecessor");
            std::string elementType;
            checker = XmlParserUtil::QueryStringAttribute(*predecessor_node,"elementType",&elementType);
            if (!checker){
                predecesor.elementType = elementType;
            }else{
                LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "Not configurate element : elementType " ;
            }

            std::string elementId;
            checker = XmlParserUtil::QueryStringAttribute(*predecessor_node,"elementId",&elementId);
            if (!checker){
                predecesor.elementId = std::stoi(elementId);
            }else{
                LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "Not configurate element : elementId " ;
            }
            std::string contactPoint;
            checker = XmlParserUtil::QueryStringAttribute(*predecessor_node,"contactPoint",&contactPoint);
            if (!checker){
                predecesor.contactPoint = contactPoint;
            }else{
                LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "Not configurate element : contactPoint " ;
            }

            Successor successor;
            const tinyxml2::XMLElement* successor_node = link_node->FirstChildElement("successor");
            std::string successor_elementType;
            checker = XmlParserUtil::QueryStringAttribute(*successor_node,"elementType",&successor_elementType);
            if (!checker){
                successor.elementType = successor_elementType;
            }else{
                LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "Not configurate element : elementType " ;
            }

            std::string successor_elementId;
            checker = XmlParserUtil::QueryStringAttribute(*successor_node,"elementId",&successor_elementId);
            if (!checker){
                successor.elementId = std::stoi(successor_elementId);
            }else{
                LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "Not configurate element : elementId " ;
            }
            std::string successor_contactPoint;
            checker = XmlParserUtil::QueryStringAttribute(*successor_node,"contactPoint",&successor_contactPoint);
            if (!checker){
                successor.contactPoint = successor_contactPoint;
            }else{
                LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "Not configurate element : contactPoint " ;
            }

            Link link;
            link.successor = successor;
            link.predecesor = predecesor;
            roadInternal.road.link = link;

            // get speed limit
            const tinyxml2::XMLElement* type_node = road_node->FirstChildElement("type");
            if (type_node){
                const tinyxml2::XMLElement* speed_node = type_node->FirstChildElement("speed");
                std::string max_str;
                checker = XmlParserUtil::QueryStringAttribute(*speed_node,"max",&max_str);
                roadInternal.road.speed_max = std::stod(max_str);
                if (!checker){
                }else{
                    LOG(WARNING)<<"["<<roadInternal.road.name<<"]" << "speed tag Not configurate element : max " ;
                }

                std::string type_str;
                checker = XmlParserUtil::QueryStringAttribute(*type_node,"type",&type_str);
                if (type_str.compare("town")){
                    roadInternal.road.type = Road::RoadType::RoadType_TOWN;
                }
            }

        }

        //get planView
        const tinyxml2::XMLElement* planview_node = road_node->FirstChildElement("planView");
        if (planview_node){
            const tinyxml2::XMLElement* geometry_node = planview_node->FirstChildElement("geometry");
            Curve reference_line;
            while (geometry_node){
                CurveSegment curveSegment;
                XmlParserUtil::ParseGeometry(*geometry_node,&curveSegment);

                std::string id_str = roadInternal.id;
                reference_line.segment.push_back(curveSegment);
                //lanes
                std::vector<RoadSectionInternal> road_section_internals;
                LaneXmlParser::Parse(*road_node,id_str,&road_section_internals,curveSegment);
//                LOG(ERROR)<< "----------------REFERECE LINE START ------------------";
//                LOG(ERROR)<< "ROAD ID : " << roadInternal.id;
//                for (int i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
//                    PointENU point = curveSegment.lineSegment.points[i];
//                    LOG(ERROR)<< point.x <<","<<point.y;
//                }
//                LOG(ERROR)<< "----------------REFERECE LINE END ------------------";
                geometry_node = geometry_node->NextSiblingElement("geometry");
            }
            //get all reference line point to one curve segment
            CurveSegment curveSegment_;
            PointENU start_point = reference_line.segment.at(0).start_position;
            curveSegment_.start_position = start_point;
            for (int i = 0; i < reference_line.segment.size(); ++i) {
                CurveSegment curveSegment__ = reference_line.segment[i];
                curveSegment_.lineSegment.points.push_back(curveSegment__.start_position);
            }

//            LOG(ERROR)<< "----------------REFERECE LINE START ------------------";
//            LOG(ERROR)<< "ROAD ID : " << roadInternal.id;
//            for (int i = 0; i < curveSegment_.lineSegment.points.size(); ++i) {
//                PointENU point = curveSegment_.lineSegment.points[i];
//                LOG(ERROR)<< point.x <<","<<point.y;
//            }
//            LOG(ERROR)<< "----------------REFERECE LINE END ------------------";
        }

        road_node = road_node->NextSiblingElement("road");
    }
    return 1;
}
}
}
}