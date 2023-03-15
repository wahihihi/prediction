//
// Created by ai on 23-3-7.
//

#ifndef PREDICTION_LANE_XML_PARSER_H
#define PREDICTION_LANE_XML_PARSER_H
#include "prediction/common_include.h"
#include "hdmap/entity/map_road.h"
#include "hdmap/parser/common_define.h"
#include "hdmap/parser/road_xml_parser.h"
using namespace aptiv::hdmap::parser;
using namespace aptiv::hdmap::entity;

namespace aptiv{
namespace hdmap{
namespace parser{
class LaneXmlParser {
public:
    LaneXmlParser() = default;
    LaneXmlParser(RoadSectionInternal roadSectionInternal_) :lastRoadSectionInternal(roadSectionInternal_){}
    int Parse(const tinyxml2::XMLElement& xml_node,
                     const std::string id,
                     RoadSectionInternal* roadSection,
                     aptiv::hdmap::entity::CurveSegment curveSegment);
    bool isFirstSection;
    RoadSectionInternal lastRoadSectionInternal;
private:
    int ParseLaneSection(const tinyxml2::XMLElement& xml_node,
                               std::vector<LaneInternal>* lanes,
                                double d_offset,
                               aptiv::hdmap::entity::CurveSegment curveSegment);

    static int ParseSectionBoundary(const tinyxml2::XMLElement& xml_node,
                                    BoundaryPolygon* boundary);
    static int ParseLane(const tinyxml2::XMLElement& xml_node,
                         MpLane* lane);
    int ParseCenterLane(const tinyxml2::XMLElement& xml_node,
                         LaneInternal* lane_internal);
    int ParseCenterCurve(LaneInternal* laneInternal,
                                double d_offset,
                                aptiv::hdmap::entity::CurveSegment curveSegment);
    static int ParseCurveArc(MpLane* lane,
                          double d_offset,
                          aptiv::hdmap::entity::CurveSegment curveSegment);
    static int ParseLeftCurve(MpLane* lane,
                            double d_offset,
                            aptiv::hdmap::entity::CurveSegment curveSegment);
    static int ParseRightCurve(MpLane* lane,
                              double d_offset,
                              aptiv::hdmap::entity::CurveSegment curveSegment);
    static int ParseLeftRoadSampleAssociates(
            const tinyxml2::XMLElement& xml_node, Lane* lane);
    static int ParseRightRoadSampleAssociates(
            const tinyxml2::XMLElement& xml_node, Lane* lane);
    static int ParseRoadRoadSampleAssociates(
            const tinyxml2::XMLElement& xml_node, Lane* lane);
    static void ParseLaneLink(const tinyxml2::XMLElement& xml_node, Lane* lane);

    int ToPbLaneType(const std::string& type, Lane::LaneType* lane_type);


};

}
}
}

#endif //PREDICTION_LANE_XML_PARSER_H
