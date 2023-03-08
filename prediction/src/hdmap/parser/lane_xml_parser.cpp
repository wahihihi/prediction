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
    std::vector<MpLaneOffSet> lane_offset_arr;
    RoadSectionInternal sectionInternal;
    while (laneOffset_node){
        laneOffset_node->QueryDoubleAttribute("s",&sOffset);
        laneOffset_node->QueryDoubleAttribute("a",&a);
        laneOffset_node->QueryDoubleAttribute("b",&b);
        laneOffset_node->QueryDoubleAttribute("c",&c);
        laneOffset_node->QueryDoubleAttribute("d",&d);
        MpLaneOffSet lane_offset;
        lane_offset.s = sOffset;
        lane_offset.a = a;
        lane_offset.b = b;
        lane_offset.c = c;
        lane_offset.d = d;

        if (sOffset > curveSegment.length){
            break;
        }
        d_offset += lane_offset.a +
                    lane_offset.b * lane_offset.s +
                    lane_offset.c * pow(lane_offset.s,2) +
                    lane_offset.d * pow(lane_offset.s,3);

        curveSegment.current_sOffset = sOffset;
        const tinyxml2::XMLElement* laneSection_node = lanes_node->FirstChildElement("laneSection");
        ParseLaneSection(*laneSection_node,&sectionInternal.lanes,d_offset,curveSegment);

        std::string section_id = std::to_string(++section_cnt);
        sectionInternal.id = section_id;
        sectionInternal.section.id = section_id;
        lane_offset_arr.push_back(lane_offset);
        laneOffset_node = laneOffset_node->NextSiblingElement("laneOffset");

        section->push_back(sectionInternal);
    }
    return 1;
}

bool compare(LaneInternal l1,LaneInternal l2){
    if (l1.lane.id < l2.lane.id){
        return true;
    }else{
        return false;
    }
};

int LaneXmlParser::ParseLaneSection(const tinyxml2::XMLElement &xml_node,
                                    std::vector<LaneInternal> *lanes,
                                    double d_offset,
                                    CurveSegment curveSegment) {
    CHECK_NOTNULL(lanes);

    // center
    const tinyxml2::XMLElement* center_node = xml_node.FirstChildElement("center");
    if (center_node) {
        LaneInternal lane_internal;
        const tinyxml2::XMLElement* lane_node = center_node->FirstChildElement("lane");
        if (lane_node) {
            ParseCenterLane(*lane_node, &lane_internal);
            // center curve
            ParseCenterCurve(&lane_internal.lane,d_offset,curveSegment);
//            LOG(ERROR)<< "----------- CENTER START --------------";
//            CurveSegment curveSegment = lane_internal.lane.central_curve.segment[0];
//            for (int i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
//                PointENU pointEnu(curveSegment.lineSegment.points[i].x,
//                                  curveSegment.lineSegment.points[i].y,
//                                  curveSegment.lineSegment.points[i].z,
//                                  curveSegment.lineSegment.points[i].s,
//                                  curveSegment.lineSegment.points[i].hdg);
//                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
//            }
//            LOG(ERROR)<< "----------- CENTER END --------------";
        }
        lane_internal.isCenter = true;
        lanes->push_back(lane_internal);
    }

    // left
    const tinyxml2::XMLElement* left_node = xml_node.FirstChildElement("left");
    if (left_node) {
        const tinyxml2::XMLElement* lane_node = left_node->FirstChildElement("lane");
        std::vector<LaneInternal> lane_internals;
        while (lane_node) {
            Lane lane_;
            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
            std::vector<LaneWidth> laneWidths;
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

//                lane_internal.lane.lane_widths.push_back(width);
                laneWidths.push_back(width);
                width_node = width_node->NextSiblingElement("width");
            }
            lane_.lane_widths = laneWidths;
            ParseLane(*lane_node, &lane_);
            LaneInternal lane_internal;
            lane_internal.lane = lane_;
            lane_internals.push_back(lane_internal);
            lane_node = lane_node->NextSiblingElement("lane");
        }
        std::sort(lane_internals.begin(),lane_internals.end(),compare);
        double acc_width = 0.0;
        for (size_t i = 0; i < lane_internals.size(); ++i) {
            Lane lane_ = lane_internals[i].lane;
            // program lane
            int idx_poly = 0;
            const double curveSegment_s = curveSegment.s;
            const double current_Offset = curveSegment.current_sOffset;
            for (size_t k = 0; k < lane_.lane_widths.size(); ++k) {
                if (k == lane_.lane_widths.size() - 1){
                    idx_poly = k;
                    break;
                }
                if (curveSegment_s >= lane_.lane_widths[k].sOffset)
                    continue;
                idx_poly = k - 1;
                break;
            }
            LaneWidth laneWidth_ = lane_.lane_widths[idx_poly];
            double sOffset = laneWidth_.sOffset;
            double ds = curveSegment_s - sOffset - sOffset;
            double calc_width = laneWidth_.a + laneWidth_.b * ds + laneWidth_.c * pow(ds,2) + laneWidth_.d * pow(ds,3);
            double total_width = acc_width + calc_width;
            // compute curve
            ParseCurve(&lane_,total_width,curveSegment);
            Curve curve = lane_.central_curve;
            LaneBoundary laneBoundary;
            laneBoundary.curve = curve;
            lane_.left_boundary = laneBoundary;
            acc_width = total_width;
//            LOG(ERROR)<< "----------- LEFT START --------------";
//            LOG(ERROR)<< "lane id : " << lane_.id;
//            std::string x_list;
//            for (int i = 0; i < lane_.left_boundary.curve.segment[0].lineSegment.points.size(); ++i) {
//                PointENU pointEnu(lane_.left_boundary.curve.segment[0].lineSegment.points[i].x,
//                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].y,
//                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].z,
//                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].s,
//                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].hdg);
//                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
//            }
//            LOG(ERROR)<< "----------- LEFT END --------------";
        }
    }


    // right
    const tinyxml2::XMLElement* right_node = xml_node.FirstChildElement("right");
    if (right_node) {
        const tinyxml2::XMLElement* lane_node = right_node->FirstChildElement("lane");
        std::vector<LaneInternal> lane_internals;
        while (lane_node) {
            Lane lane_;
            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
            std::vector<LaneWidth> laneWidths;
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

//                lane_internal.lane.lane_widths.push_back(width);
                laneWidths.push_back(width);
                width_node = width_node->NextSiblingElement("width");
            }
            lane_.lane_widths = laneWidths;
            ParseLane(*lane_node, &lane_);
            LaneInternal lane_internal;
            lane_internal.lane = lane_;
            lane_internals.push_back(lane_internal);
            lane_node = lane_node->NextSiblingElement("lane");
        }
        std::sort(lane_internals.begin(),lane_internals.end(),compare);
        double acc_width = 0.0;
        for (size_t i = 0; i < lane_internals.size(); ++i) {
            Lane lane_ = lane_internals[i].lane;
            // program lane
            int idx_poly = 0;
            double curveSegment_s = curveSegment.s;
            double current_sOffset = curveSegment.current_sOffset;
            for (size_t k = 0; k < lane_.lane_widths.size(); ++k) {
                if (k == lane_.lane_widths.size() - 1){
                    idx_poly = k;
                    break;
                }
                if (curveSegment_s >= lane_.lane_widths[k].sOffset)
                    continue;
                idx_poly = k - 1;
                break;
            }
            LaneWidth laneWidth_ = lane_.lane_widths[idx_poly];
            double sOffset = laneWidth_.sOffset;
            double ds = curveSegment_s - current_sOffset - sOffset;
            double calc_width = laneWidth_.a + laneWidth_.b * ds + laneWidth_.c * pow(ds,2) + laneWidth_.d * pow(ds,3);
            double total_width = acc_width - calc_width;
            // compute curve
            ParseCurve(&lane_,total_width,curveSegment);
            Curve curve = lane_.central_curve;
            LaneBoundary laneBoundary;
            laneBoundary.curve = curve;
            lane_.left_boundary = laneBoundary;
            acc_width = total_width;
            LOG(ERROR)<< "----------- RIGHT START --------------";
            LOG(ERROR)<< "lane id : " << lane_.id;
            std::string x_list;
            for (int i = 0; i < lane_.left_boundary.curve.segment[0].lineSegment.points.size(); ++i) {
                PointENU pointEnu(lane_.left_boundary.curve.segment[0].lineSegment.points[i].x,
                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].y,
                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].z,
                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].s,
                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].hdg);
                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
            }
            LOG(ERROR)<< "----------- RIGHT END --------------";
        }
    }
    return 1;
}

int LaneXmlParser::ParseLane(const tinyxml2::XMLElement& xml_node,Lane* lane) {
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

    return 1;
}

int LaneXmlParser::ParseCenterLane(const tinyxml2::XMLElement& xml_node,
                                 LaneInternal* lane_internal) {
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

int LaneXmlParser::ParseCurve(aptiv::hdmap::parser::Lane *lane,
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
    }
    lane->central_curve.segment.push_back(res_curveSegment);
    return 1;
}

int LaneXmlParser::ParseCenterCurve(aptiv::hdmap::parser::Lane *lane,
                                    double d_offset,
                                    CurveSegment curveSegment) {
    CurveSegment res_curveSegment;
    for (size_t j = 0; j < curveSegment.lineSegment.points.size(); ++j) {
        PointENU pointEnu = curveSegment.lineSegment.points[j];
        const double x = pointEnu.x - d_offset * sin(pointEnu.hdg);
        const double y = pointEnu.y - d_offset * cos(pointEnu.hdg);
        const double s = hypot(x,y);
        PointENU point(x,y,0,s,pointEnu.hdg);
        res_curveSegment.lineSegment.points.push_back(point);
    }
    lane->central_curve.segment.push_back(res_curveSegment);
    return 1;
}

}
}
}
