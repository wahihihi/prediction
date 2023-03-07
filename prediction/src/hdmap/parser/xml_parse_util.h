//
// Created by ai on 23-3-7.
//

#ifndef PREDICTION_XML_PARSE_UTIL_H
#define PREDICTION_XML_PARSE_UTIL_H
#include "prediction/common_include.h"
#include "hdmap/entity/map_geometry.h"


namespace aptiv {
namespace hdmap {
namespace parser {

using namespace aptiv::hdmap::entity;

class XmlParserUtil {
public:
static tinyxml2::XMLError QueryStringAttribute(
        const tinyxml2::XMLElement &xml_node, const std::string &name,
        std::string *value);

static std::string ToUpper(const std::string& s);
static int ParseGeometry(const tinyxml2::XMLElement& xml_node,CurveSegment* curveSegment);
static int ParseCurve(const tinyxml2::XMLElement& xml_node,CurveSegment* curve_segment);
static int ParsePointSet(const CurveSegment& curveSegment,LineSegment* line_segment);
static int ParsePointSet(const CurveSegment& curveSegment,LineSegment* line_segment,double curvature);

};


}
}
}


#endif //PREDICTION_XML_PARSE_UTIL_H
