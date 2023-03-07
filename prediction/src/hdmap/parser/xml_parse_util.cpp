//
// Created by ai on 23-3-7.
//

#include "xml_parse_util.h"
namespace aptiv {
namespace hdmap {
namespace parser {

tinyxml2::XMLError XmlParserUtil::QueryStringAttribute(const tinyxml2::XMLElement &xml_node, const std::string &name,
                                                       std::string *value) {
    const char* val = xml_node.Attribute(name.c_str());
    if (val == nullptr) {
        return tinyxml2::XML_NO_ATTRIBUTE;
    }

    *value = val;
    return tinyxml2::XML_SUCCESS;
}

std::string XmlParserUtil::ToUpper(const std::string& s) {
    std::string value = s;
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    return value;
}

int XmlParserUtil::ParseGeometry(const tinyxml2::XMLElement& xml_node,
                                 CurveSegment* curveSegment) {
    CHECK_NOTNULL(curveSegment);
    XmlParserUtil::ParseCurve(xml_node, curveSegment);
    return 0;
}

int XmlParserUtil::ParseCurve(const tinyxml2::XMLElement& xml_node,
                                    CurveSegment* curve_segment) {
    CHECK_NOTNULL(curve_segment);

    // Read geometry attributes
    double s = 0.0;
    double ptx = 0.0;
    double pty = 0.0;
    double ptz = 0.0;
    double hdg = 0.0;
    double length = 0.0;

    int checker = tinyxml2::XML_SUCCESS;

    checker += xml_node.QueryDoubleAttribute("s", &s);
    checker += xml_node.QueryDoubleAttribute("x", &ptx);
    checker += xml_node.QueryDoubleAttribute("y", &pty);
    checker += xml_node.QueryDoubleAttribute("hdg", &hdg);
    checker += xml_node.QueryDoubleAttribute("length", &length);

    if (checker == tinyxml2::XML_SUCCESS) {


        const tinyxml2::XMLElement* line_node = xml_node.FirstChildElement("line");
        if (line_node){
            curve_segment->s = s;

            double output_x = 0.0;
            double output_y = 0.0;
            double output_z = 0.0;

            PointENU pointEnu(ptx,pty,ptz,s,hdg);
//        WGS84ToUTM(ptx, pty, ptz, &output_x, &output_y, &output_z);
            curve_segment->start_position = pointEnu;
            curve_segment->length = length;
            curve_segment->heading = hdg;
            ParsePointSet(*curve_segment, &curve_segment->lineSegment);
            return 0;
        } else {
            const tinyxml2::XMLElement* arc_node = xml_node.FirstChildElement("arc");
            if (arc_node){
                double curvature;

                int checker = tinyxml2::XML_SUCCESS;
                checker += xml_node.QueryDoubleAttribute("curvature", &curvature);
                ParsePointSet(*curve_segment, &curve_segment->lineSegment,curvature);
                return 0;
            }
        }
    }

    std::string err_msg = "Error geometry object";
    LOG(ERROR) << err_msg;
    return -1;
}



int XmlParserUtil::ParsePointSet(const aptiv::hdmap::entity::CurveSegment &curveSegment,
                                 aptiv::hdmap::entity::LineSegment *line_segment) {

    double delta_s = 0.2;
    double s = curveSegment.s;
    int sample_num = int(curveSegment.length/delta_s);

    for (int i = 0; i < sample_num; ++i) {
        double x = curveSegment.start_position.x + (delta_s * i) * cos(curveSegment.heading);
        double y = curveSegment.start_position.y + (delta_s * i) * sin(curveSegment.heading);
        s += delta_s;
        PointENU point(x,y,0,s,curveSegment.heading);
        line_segment->points.push_back(point);
    }
}

int XmlParserUtil::ParsePointSet(const aptiv::hdmap::entity::CurveSegment &curveSegment,
                                 aptiv::hdmap::entity::LineSegment *line_segment,
                                 double curvature) {

    double delta_s = 0.2;
    double x = curveSegment.start_position.x;
    double y = curveSegment.start_position.y;
    double s = curveSegment.s;
    double radius = 1 / curvature;
    double hdg = curveSegment.heading;
    double sin_hdg = sin(curveSegment.heading);
    double cos_hdg = cos(curveSegment.heading);
    int sample_num = int(curveSegment.length/delta_s);
    for (int i = 0; i < sample_num; ++i) {
        const double angle_at_s = delta_s * curvature - M_PI / 2;
        const double xd = radius * (cos(hdg + angle_at_s) - sin_hdg) + x;
        const double yd = radius * (sin(hdg + angle_at_s) + cos_hdg) + y;
        const double tangent = hdg + delta_s * curvature;
        s += delta_s;
        PointENU pointEnu(xd,yd,0,s,tangent);
        line_segment->points.push_back(pointEnu);
    }

}
}
}
}