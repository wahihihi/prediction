//
// Created by ai on 23-3-7.
//

#include "lane_xml_parser.h"
#include <iostream>

namespace aptiv{
namespace hdmap{
namespace parser{

int LaneXmlParser::Parse(const tinyxml2::XMLElement &xml_node,
                         const std::string id,
                         std::vector<RoadSectionInternal>* roadSections,
                         CurveSegment curveSegment) {
    CHECK_NOTNULL(roadSections);
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

        d_offset += lane_offset.a +
                    lane_offset.b * lane_offset.s +
                    lane_offset.c * pow(lane_offset.s,2) +
                    lane_offset.d * pow(lane_offset.s,3);

        // split curve segment by sOffset
        CurveSegment lane_curve_segment;
        PointENU start_point;
        start_point = curveSegment.start_position;
        RoadSectionInternal road_section_internal;
        LineSegment line_segment;
        lane_curve_segment.lineSegment    = curveSegment.lineSegment;
        lane_curve_segment.s              = curveSegment.s;
        lane_curve_segment.length         = curveSegment.length;
        lane_curve_segment.curveType        = curveSegment.curveType;
        lane_curve_segment.laneOffset     = lane_offset.s;
        lane_curve_segment.start_position = start_point;
        lane_curve_segment.heading        = start_point.hdg;

        if (lane_offset.s >= lane_curve_segment.s){
            std::vector<LaneInternal> lanes;
            const tinyxml2::XMLElement* laneSection_node = lanes_node->FirstChildElement("laneSection");
            ParseLaneSection(*laneSection_node,&lanes,d_offset,lane_curve_segment);
            road_section_internal.lanes = lanes;
            roadSections->push_back(road_section_internal);
            line_segment.points.erase(line_segment.points.begin(),line_segment.points.end());
            break;
        }else{
            laneOffset_node = laneOffset_node->NextSiblingElement("laneOffset");
        }

//        std::string section_id = std::to_string(++section_cnt);
//        sectionInternal.id = section_id;
//        sectionInternal.section.id = section_id;
//        lane_offset_arr.push_back(lane_offset);
//
//        section->push_back(sectionInternal);
    }

    LOG(ERROR) << "--------------- CENTER LINE START -----------------";
    for (int i = 0; i < roadSections->size(); ++i) {
        RoadSectionInternal roadSectionInternal = roadSections->at(i);
        for (int j = 0; j < roadSectionInternal.lanes.size(); ++j) {
            LaneInternal laneInternal = roadSectionInternal.lanes.at(j);
            MpLane lane = laneInternal.lane;
            Curve curve = lane.central_curve;
            for (int k = 0; k < curve.segment.size(); ++k) {
                CurveSegment curveSegment1 = curve.segment.at(k);
                for (int l = 0; l < curveSegment1.lineSegment.points.size(); ++l) {
                    PointENU pointEnu = curveSegment1.lineSegment.points.at(l);
                    LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
                }
            }
        }
    }
    LOG(ERROR) << "--------------- CENTER LINE END -----------------";
    return 1;
}

bool compare(LaneInternal l1,LaneInternal l2){
    if (std::stoi(l1.lane.id) > std::stoi(l2.lane.id)){
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
            std::shared_ptr<MpLane> lane_ptr_(new MpLane());
            ParseCenterCurve(&lane_internal,d_offset,curveSegment);
            LOG(ERROR)<< "----------- CENTER START --------------";
            CurveSegment curveSegment_ = lane_internal.lane.central_curve.segment[0];
            for (int i = 0; i < curveSegment_.lineSegment.points.size(); ++i) {
                PointENU pointEnu(curveSegment_.lineSegment.points[i].x,
                                  curveSegment_.lineSegment.points[i].y,
                                  curveSegment_.lineSegment.points[i].z,
                                  curveSegment_.lineSegment.points[i].s,
                                  curveSegment_.lineSegment.points[i].hdg);
                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
            }
            LOG(ERROR)<< "----------- CENTER END --------------";
        }
        lane_internal.isCenter = true;
        lanes->push_back(lane_internal);
    }

    // left
//    const tinyxml2::XMLElement* left_node = xml_node.FirstChildElement("left");
//    if (left_node) {
//        const tinyxml2::XMLElement* lane_node = left_node->FirstChildElement("lane");
//        while (lane_node) {
//            MpLane lane;
//            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
//            std::vector<LaneWidth> laneWidths;
//            while (width_node){
//                LaneWidth width;
//                double sOffset_=0.0;
//                double a=0.0;
//                double b=0.0;
//                double c=0.0;
//                double d=0.0;
//                width_node->QueryDoubleAttribute("sOffset",&sOffset_);
//                width_node->QueryDoubleAttribute("a",&a);
//                width_node->QueryDoubleAttribute("b",&b);
//                width_node->QueryDoubleAttribute("c",&c);
//                width_node->QueryDoubleAttribute("d",&d);
//                width.sOffset = sOffset_;
//                width.a = a;
//                width.b = b;
//                width.c = c;
//                width.d = d;
//
////                lane_internal.lane.lane_widths.push_back(width);
//                laneWidths.push_back(width);
//                width_node = width_node->NextSiblingElement("width");
//            }
//            lane.lane_widths = laneWidths;
//            ParseLane(*lane_node, &lane);
//            LaneInternal lane_internal;
//            lane_internal.lane = lane;
//            lanes->push_back(lane_internal);
//            lane_node = lane_node->NextSiblingElement("lane");
//        }
//        std::sort(lanes->begin(),lanes->end(),compare);
//        for (size_t i = 0; i < lanes->size(); ++i) {
//            MpLane lane = lanes->at(i).lane;
//            // program lane
//            int idx_poly = 0;
//            const double curveSegment_s = curveSegment.s;
//            const double curveSegment_laneOffset = curveSegment.laneOffset;
//            for (size_t k = 0; k < lane.lane_widths.size(); ++k) {
//                if (k == lane.lane_widths.size() - 1){
//                    idx_poly = k;
//                    break;
//                }
//                if (curveSegment_s >= lane.lane_widths[k].sOffset)
//                    continue;
//                idx_poly = k - 1;
//                break;
//            }
//            LaneWidth laneWidth_ = lane.lane_widths[idx_poly];
//            double sOffset = laneWidth_.sOffset;
//            double ds = curveSegment_s - curveSegment_laneOffset - sOffset;
//            double calc_width = laneWidth_.a + laneWidth_.b * ds + laneWidth_.c * pow(ds,2) + laneWidth_.d * pow(ds,3);
//            // compute curve
//            ParseCurve(&lane,calc_width,curveSegment);
//            Curve curve = lane.central_curve;
//            LaneBoundary laneBoundary;
//            laneBoundary.curve = curve;
//            lane.left_boundary = laneBoundary;
////            LOG(ERROR)<< "----------- LEFT START --------------";
////            LOG(ERROR)<< "lane id : " << lane_.id;
////            std::string x_list;
////            for (int i = 0; i < lane_.left_boundary.curve.segment[0].lineSegment.points.size(); ++i) {
////                PointENU pointEnu(lane_.left_boundary.curve.segment[0].lineSegment.points[i].x,
////                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].y,
////                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].z,
////                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].s,
////                                  lane_.left_boundary.curve.segment[0].lineSegment.points[i].hdg);
////                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
////            }
////            LOG(ERROR)<< "----------- LEFT END --------------";
//        }
//    }

    // right
//    std::vector<LaneInternal> lane_internals;
//    const tinyxml2::XMLElement* right_node = xml_node.FirstChildElement("right");
//    if (right_node) {
//        const tinyxml2::XMLElement* lane_node = right_node->FirstChildElement("lane");
//        while (lane_node) {
//            MpLane lane;
//            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
//            std::vector<LaneWidth> laneWidths;
//            while (width_node){
//                LaneWidth width;
//                double sOffset_=0.0;
//                double a=0.0;
//                double b=0.0;
//                double c=0.0;
//                double d=0.0;
//                width_node->QueryDoubleAttribute("sOffset",&sOffset_);
//                width_node->QueryDoubleAttribute("a",&a);
//                width_node->QueryDoubleAttribute("b",&b);
//                width_node->QueryDoubleAttribute("c",&c);
//                width_node->QueryDoubleAttribute("d",&d);
//                width.sOffset = sOffset_;
//                width.a = a;
//                width.b = b;
//                width.c = c;
//                width.d = d;
//
////                lane_internal.lane.lane_widths.push_back(width);
//                laneWidths.push_back(width);
//                width_node = width_node->NextSiblingElement("width");
//            }
//            lane.lane_widths = laneWidths;
//            ParseLane(*lane_node, &lane);
//            LaneInternal lane_internal;
//            lane_internal.lane = lane;
//            lane_internals.push_back(lane_internal);
//            lane_node = lane_node->NextSiblingElement("lane");
//        }
//        std::sort(lane_internals.begin(),lane_internals.end(),compare);
//        for (size_t i = 0; i < lane_internals.size(); ++i) {
//            MpLane* lane = &lane_internals.at(i).lane;
//            // program lane
//            int idx_poly = 0;
//            double curveSegment_s = curveSegment.s;
//            double curveSegment_laneOffset = curveSegment.laneOffset;
//            for (size_t k = 0; k < lane->lane_widths.size(); ++k) {
//                if (k == lane->lane_widths.size() - 1){
//                    idx_poly = k;
//                    break;
//                }
//                if (curveSegment_s >= lane->lane_widths[k].sOffset)
//                    continue;
//                idx_poly = k - 1;
//                break;
//            }
//            LaneWidth laneWidth_ = lane->lane_widths[idx_poly];
//            double sOffset = laneWidth_.sOffset;
//            double ds = curveSegment_s - curveSegment_laneOffset - sOffset;
//            double calc_width = laneWidth_.a + laneWidth_.b * ds + laneWidth_.c * pow(ds,2) + laneWidth_.d * pow(ds,3);
//            // compute curve
//            ParseCurveArc(lane,calc_width,curveSegment);
//            lane->left_boundary.curve = lane->central_curve;
//
//            LOG(ERROR)<< "----------- RIGHT START --------------";
//            LOG(ERROR)<< "lane id : " << lane->id;
//            for (int i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
//                PointENU pointEnu(curveSegment.lineSegment.points[i].x,
//                                  curveSegment.lineSegment.points[i].y,
//                                  curveSegment.lineSegment.points[i].z,
//                                  curveSegment.lineSegment.points[i].s,
//                                  curveSegment.lineSegment.points[i].hdg);
//                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
//            }
//
//            LOG(ERROR) << "==================================";
//            for (int i = 0; i < lane->left_boundary.curve.segment[0].lineSegment.points.size(); ++i) {
//                PointENU pointEnu(lane->left_boundary.curve.segment[0].lineSegment.points[i].x,
//                                  lane->left_boundary.curve.segment[0].lineSegment.points[i].y,
//                                  lane->left_boundary.curve.segment[0].lineSegment.points[i].z,
//                                  lane->left_boundary.curve.segment[0].lineSegment.points[i].s,
//                                  lane->left_boundary.curve.segment[0].lineSegment.points[i].hdg);
//                LOG(ERROR) << pointEnu.x << "," << pointEnu.y;
//            }
//            LOG(ERROR)<< "----------- RIGHT END --------------";
//            lanes->push_back(lane_internals.at(i));
//        }
//    }
    return 1;
}

int LaneXmlParser::ParseLane(const tinyxml2::XMLElement& xml_node,MpLane* lane) {
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

    MpLane lane;
    // lane id
    std::string lane_id;
    int checker = XmlParserUtil::QueryStringAttribute(xml_node,"id", &lane_id);
    if (checker != tinyxml2::XML_SUCCESS) {
        std::string err_msg = "Error parse lane id";
        LOG(ERROR) << err_msg;
        return 1;
    }

    lane.id = lane_id ;

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
    lane.laneType = e_lane_type;

    lane_internal->lane = lane;
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
int LaneXmlParser::ParseCurve(MpLane* lane,
                              double d_offset,
                              CurveSegment curveSegment) {

    CurveSegment res_curveSegment;
    for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
        PointENU pointEnu = curveSegment.lineSegment.points[i];
        const double x = pointEnu.x - d_offset * sin(pointEnu.hdg);
        const double y = pointEnu.y - d_offset * cos(pointEnu.hdg);
        const double s = pointEnu.s;
        PointENU point(x,y,0,pointEnu.s,pointEnu.hdg);
        res_curveSegment.lineSegment.points.push_back(point);
    }
    lane->central_curve.segment.push_back(res_curveSegment);
    return 1;
}

int LaneXmlParser::ParseCurveArc(MpLane* lane,
                                double d_offset,
                                CurveSegment curveSegment) {

    CurveSegment res_curveSegment;
    LOG(ERROR) << "------------------CURVE ARC START------------------";
    for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
        PointENU pointEnu = curveSegment.lineSegment.points[i];
        double sin_hdg = sin(pointEnu.hdg);
        double cos_hdg = cos(pointEnu.hdg);
        const double x = pointEnu.x - d_offset/2 * sin_hdg;
        const double y = pointEnu.y + d_offset/2 * cos_hdg;
        const double s = pointEnu.s;
        PointENU point(x,y,0,pointEnu.s,pointEnu.hdg);
        res_curveSegment.lineSegment.points.push_back(point);
//        LOG(ERROR) << point.x << "," << point.y ;

    }
    LOG(ERROR) << "------------------CURVE ARC END------------------";
    lane->central_curve.segment.push_back(res_curveSegment);
    return 1;
}

int LaneXmlParser::ParseCenterCurve(LaneInternal* laneInternal,
                                    double d_offset,
                                    CurveSegment curveSegment) {
    CurveSegment res_curveSegment;
    double delta_s = 0.2;
    PointENU start_point = curveSegment.start_position;
    PointENU internal_start_point;
    double s = curveSegment.s;
    double x = curveSegment.start_position.x;
    double y = curveSegment.start_position.y;
    double hdg = curveSegment.start_position.hdg;
    double cos_hdg = cos(hdg);
    double sin_hdg = sin(hdg);
    if (curveSegment.curveType == entity::CurveSegment::CurveType_Line){
        double internal_start_x = x - d_offset * cos_hdg;
        double internal_start_y = y + d_offset * sin_hdg;
        internal_start_point.y = internal_start_x;
        internal_start_point.z = internal_start_y;
        internal_start_point.s = start_point.s;
        internal_start_point.hdg = hdg;
        for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
            double dx = internal_start_point.x + (delta_s * i) * cos_hdg;
            double dy = internal_start_point.y + (delta_s * i) * sin_hdg;
            PointENU point(dx,dy,0,s,curveSegment.heading);
            s += delta_s;
            res_curveSegment.lineSegment.points.push_back(point);
        }
    }else if (curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
        double internal_start_x = x - d_offset * cos_hdg;
        double internal_start_y = y + d_offset * sin_hdg;
        internal_start_point.x = internal_start_x;
        internal_start_point.y = internal_start_y;
        internal_start_point.s = start_point.s;
        internal_start_point.hdg = hdg;
        double curvature = start_point.curveture;
        double radius = 1 / curvature;
        for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
            const double ref_line_ds = delta_s * i;
            const double angle_at_s = ref_line_ds * curvature - M_PI / 2;
            const double xd = radius * (cos(hdg + angle_at_s) - sin_hdg) + x;
            const double yd = radius * (sin(hdg + angle_at_s) + cos_hdg) + y;
            const double tangent = hdg + delta_s * curvature;
            PointENU* pointEnu = new PointENU(xd,yd,0,s,tangent);
            s += delta_s;
            pointEnu->curveture = curvature;
            res_curveSegment.lineSegment.points.push_back(*pointEnu);
        }
    }
    laneInternal->lane.central_curve.segment.push_back(res_curveSegment);
    return 1;
}

}
}
}
