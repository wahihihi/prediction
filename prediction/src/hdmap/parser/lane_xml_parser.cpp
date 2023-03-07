//
// Created by ai on 23-3-7.
//

#include "lane_xml_parser.h"

namespace aptiv{
namespace hdmap{
namespace parser{

int LaneXmlParser::Parse(const tinyxml2::XMLElement &xml_node,
                         const std::string id,
                         std::vector<RoadSectionInternal> *section,
                         CurveSegment curveSegment) {
    CHECK_NOTNULL(section);
    LOG(ERROR) << ">>>>>>>>>>>>>>>> lane xml parser run";

    const auto lanes_node = xml_node.FirstChildElement("lanes");
    CHECK_NOTNULL(lanes_node);
    const tinyxml2::XMLElement* laneOffset_node = lanes_node->FirstChildElement("laneOffset");
    CHECK_NOTNULL(laneOffset_node);
    size_t section_cnt = 0;
    double sOffset =0.0;
    double a =0.0;
    double b =0.0;
    double c =0.0;
    double d =0.0;
    double d_offset =0.0;
    while (laneOffset_node){
        RoadSectionInternal sectionInternal;
        laneOffset_node->QueryDoubleAttribute("s",&sOffset);
        laneOffset_node->QueryDoubleAttribute("a",&a);
        laneOffset_node->QueryDoubleAttribute("b",&b);
        laneOffset_node->QueryDoubleAttribute("c",&c);
        laneOffset_node->QueryDoubleAttribute("d",&d);
        d_offset += a + b * sOffset + c * pow(sOffset,2) + d * pow(sOffset,3);


        std::string section_id = std::to_string(++section_cnt);
        sectionInternal.id = section_id;
        sectionInternal.section.id = section_id;
        const tinyxml2::XMLElement* laneSection_node = lanes_node->FirstChildElement("laneSection");
        ParseLaneSection(*laneSection_node,&sectionInternal.lanes,sOffset,d_offset,curveSegment);
        laneOffset_node = laneOffset_node->NextSiblingElement("laneOffset");
        section->push_back(sectionInternal);
    }
    return 1;
}

int LaneXmlParser::ParseLaneSection(const tinyxml2::XMLElement &xml_node,
                                    std::vector<LaneInternal> *lanes,
                                    double s_section,
                                    double d_offset,
                                    CurveSegment curveSegment) {
    CHECK_NOTNULL(lanes);

    LaneInternal lane_internal;
    // center
    const tinyxml2::XMLElement* center_node = xml_node.FirstChildElement("center");
    if (center_node) {
        const tinyxml2::XMLElement* lane_node = center_node->FirstChildElement("lane");
        if (lane_node) {
            ParseCenterLane(*lane_node, &lane_internal,s_section,d_offset,curveSegment);
            LOG(ERROR)<< "----------- CENTER START --------------";
            CurveSegment curveSegment = lane_internal.lane.central_curve.segment[0];
            for (int i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
                PointENU pointEnu(curveSegment.lineSegment.points[i].x,
                                  curveSegment.lineSegment.points[i].y,
                                  curveSegment.lineSegment.points[i].z,
                                  curveSegment.lineSegment.points[i].s,
                                  curveSegment.lineSegment.points[i].hdg);
                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
            }
            LOG(ERROR)<< "----------- CENTER END --------------";
        }
    }

    // left
    const tinyxml2::XMLElement* left_node = xml_node.FirstChildElement("left");
    if (left_node) {
        const tinyxml2::XMLElement* lane_node = left_node->FirstChildElement("lane");
        while (lane_node) {
            std::vector<LaneWidth> laneWidth;
            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
            double s_section_ = s_section;
            while (width_node){
                LaneWidth width;
                double sOffset_=0.0;
                double a=0.0;
                double b=0.0;
                double c=0.0;
                double d=0.0;
                width_node->QueryDoubleAttribute("sOffset",&sOffset_);
                width_node->QueryDoubleAttribute("a",&a);
                width_node->QueryDoubleAttribute("b",&b);
                width_node->QueryDoubleAttribute("c",&c);
                width_node->QueryDoubleAttribute("d",&d);
                width.sOffset = sOffset_;
                width.a = a;
                width.b = b;
                width.c = c;
                width.d = d;

                lane_internal.lane.lane_widths.push_back(width);
                width_node = width_node->NextSiblingElement("width");
            }
            int idx_poly = 0;
            double curveSegment_s = curveSegment.s;
            for (size_t k = 0; k < lane_internal.lane.lane_widths.size(); ++k) {
                if (k == lane_internal.lane.lane_widths.size() - 1){
                    idx_poly = k;
                    break;
                }
                if (curveSegment_s >= lane_internal.lane.lane_widths[k].sOffset)
                    continue;
                idx_poly = k - 1;
                break;
            }
            LaneWidth laneWidth_ = lane_internal.lane.lane_widths[idx_poly];
            double sOffset = laneWidth_.sOffset;
            double ds = curveSegment_s - s_section_ - sOffset;
            double calc_width = laneWidth_.a + laneWidth_.b * ds + laneWidth_.c * pow(ds,2) + laneWidth_.d * pow(ds,3);

            ParseLane(*lane_node, &lane_internal,s_section,calc_width,curveSegment);
            LOG(ERROR)<< "----------- LEFT START --------------";
            for (int i = 0; i < lane_internal.lane.left_boundary.curve.segment[0].lineSegment.points.size(); ++i) {
                PointENU pointEnu(lane_internal.lane.left_boundary.curve.segment[0].lineSegment.points[i].x,
                                  lane_internal.lane.left_boundary.curve.segment[0].lineSegment.points[i].y,
                                  lane_internal.lane.left_boundary.curve.segment[0].lineSegment.points[i].z,
                                  lane_internal.lane.left_boundary.curve.segment[0].lineSegment.points[i].s,
                                  lane_internal.lane.left_boundary.curve.segment[0].lineSegment.points[i].hdg);
                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
            }
            LOG(ERROR)<< "----------- LEFT END --------------";
            lanes->push_back(lane_internal);
            lane_node = lane_node->NextSiblingElement("lane");
        }
    }
    return 1;
}

int LaneXmlParser::ParseLane(const tinyxml2::XMLElement& xml_node,
                                   LaneInternal* lane_internal,
                                   double sOffset,
                                   double d_offset,
                                   CurveSegment curveSegment) {
    CHECK_NOTNULL(lane_internal);

    Lane* lane = &lane_internal->lane;
    // lane id
    std::string lane_id;
    int checker = XmlParserUtil::QueryStringAttribute(xml_node,"id", &lane_id);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane id";
        LOG(ERROR) << err_msg;
        return 0;
    }
    lane->id = lane_id;

    // lane type
    std::string lane_type;
    checker = XmlParserUtil::QueryStringAttribute(xml_node, "type", &lane_type);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane type.";
        LOG(ERROR) << err_msg;
        return 0;
    }
    Lane::LaneType e_lane_type;
    int success = ToPbLaneType(lane_type, &e_lane_type);
    if (!success) {
        std::string err_msg = "Error convert lane type to pb lane type.";
        return 0;
    }
    lane->laneType = e_lane_type;

    // compute curve
    ParseCurve(xml_node, lane,sOffset,d_offset,curveSegment);
    Curve curve;
    curve.segment.push_back(curveSegment) ;
    LaneBoundary laneBoundary;
    laneBoundary.curve = curve;
    lane->left_boundary = laneBoundary;

    return 1;
}

int LaneXmlParser::ParseCenterLane(const tinyxml2::XMLElement& xml_node,
                                 LaneInternal* lane_internal,
                                double sOffset,
                                double d_offset,
                                 CurveSegment curveSegment) {
    CHECK_NOTNULL(lane_internal);

    Lane* lane = &lane_internal->lane;
    // lane id
    std::string lane_id;
    int checker = XmlParserUtil::QueryStringAttribute(xml_node,"id", &lane_id);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane id";
        LOG(ERROR) << err_msg;
        return 1;
    }
    lane->id = lane_id;

    // lane type
    std::string lane_type;
    checker = XmlParserUtil::QueryStringAttribute(xml_node, "type", &lane_type);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane type.";
        LOG(ERROR) << err_msg;
        return 0;
    }
    Lane::LaneType e_lane_type;
    int success = ToPbLaneType(lane_type, &e_lane_type);
    if (!success) {
        std::string err_msg = "Error convert lane type to pb lane type.";
        return 0;
    }
    lane->laneType = e_lane_type;

    // center curve
    ParseCenterCurve(xml_node, lane,sOffset,d_offset,curveSegment);

    return 1;
}


int LaneXmlParser::ToPbLaneType(const std::string& type,
                                    Lane::LaneType* lane_type)
{
    CHECK_NOTNULL(lane_type);

    std::string upper_str = XmlParserUtil::ToUpper(type);

    if (upper_str == "NONE") {
        *lane_type = Lane::LaneType::LaneType_None;
    } else if (upper_str == "DRIVING") {
        *lane_type = Lane::LaneType::LaneType_Driving;
    } else if (upper_str == "BIKING") {
        *lane_type = Lane::LaneType::LaneType_Biking;
    } else if (upper_str == "PARKING") {
        *lane_type = Lane::LaneType::LaneType_Parking;
    } else if (upper_str == "SHOULDER") {
        *lane_type = Lane::LaneType::LaneType_Shoulder;
    } else if (upper_str == "SIDEWALK") {
        *lane_type = Lane::LaneType::LaneType_Sidwalk;
    } else if (upper_str == "MEDIAN") {
        *lane_type = Lane::LaneType::LaneType_Sidwalk;
    } else {
        std::string err_msg = "Error or unsupport lane type:" + type;
        LOG(ERROR) << err_msg;
        return 0;
    }

    return 1;
}

int LaneXmlParser::ParseCurve(const tinyxml2::XMLElement &xml_node,
                                    aptiv::hdmap::parser::Lane *lane,
                                    double sOffset,
                                    double d_offset,
                                    CurveSegment curveSegment) {

    CurveSegment res_curveSegment;
    for (size_t j = 0; j < curveSegment.lineSegment.points.size(); ++j) {
        PointENU pointEnu = curveSegment.lineSegment.points[j];
        const double x = pointEnu.x - d_offset * sin(pointEnu.hdg);
        const double y = pointEnu.y - d_offset * cos(pointEnu.hdg);
        const double s = pointEnu.s;
        PointENU point(x,y,0,pointEnu.s,pointEnu.hdg);
        res_curveSegment.lineSegment.points.push_back(point);
        if (point.s >= sOffset){
            lane->central_curve.segment.push_back(res_curveSegment);
            return 1;
        }
    }
}

int LaneXmlParser::ParseCenterCurve(const tinyxml2::XMLElement &xml_node,
                                    aptiv::hdmap::parser::Lane *lane,
                                    double sOffset,
                                    double d_offset,
                                    CurveSegment curveSegment) {
    CurveSegment res_curveSegment;
    for (size_t j = 0; j < curveSegment.lineSegment.points.size(); ++j) {
        PointENU pointEnu = curveSegment.lineSegment.points[j];
        const double x = pointEnu.x - d_offset * sin(pointEnu.hdg);
        const double y = pointEnu.y - d_offset * cos(pointEnu.hdg);
        const double s = pointEnu.s;
        PointENU point(x,y,0,pointEnu.s,pointEnu.hdg);
        if (point.s <= sOffset){
            res_curveSegment.lineSegment.points.push_back(point);
        } else {
            lane->central_curve.segment.push_back(res_curveSegment);
            return 1;
        }
    }
}

}
}
}
