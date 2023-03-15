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
                         RoadSectionInternal* roadSection,
                         CurveSegment curveSegment) {
    CHECK_NOTNULL(roadSection);
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
    size_t offset_idx = 0;
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

        d_offset = lane_offset.a +
                    lane_offset.b * lane_offset.s +
                    lane_offset.c * pow(lane_offset.s,2) +
                    lane_offset.d * pow(lane_offset.s,3);

        // split curve segment by sOffset
        CurveSegment lane_curve_segment;
        PointENU start_point;
        start_point = curveSegment.start_position;
//        RoadSectionInternal road_section_internal;
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
//            road_section_internal.lanes = lanes;
            roadSection->lanes = lanes;
            break;
        }else{
            ++offset_idx;
            laneOffset_node = laneOffset_node->NextSiblingElement("laneOffset");
        }
    }
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
    LaneInternal center_lane_internal;
    if (center_node) {
        const tinyxml2::XMLElement* lane_node = center_node->FirstChildElement("lane");
        if (lane_node) {
            ParseCenterLane(*lane_node, &center_lane_internal);
            // center curve
            std::shared_ptr<MpLane> lane_ptr_(new MpLane());
            ParseCenterCurve(&center_lane_internal,d_offset,curveSegment);
        }
        center_lane_internal.isCenter = true;
        lanes->push_back(center_lane_internal);
    }

//     left
//    const tinyxml2::XMLElement* left_node = xml_node.FirstChildElement("left");
//    if (left_node) {
//        LaneInternal lane_internal;
//        const tinyxml2::XMLElement* lane_node = left_node->FirstChildElement("lane");
//        while (lane_node) {
//            MpLane lane;
//            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
//            std::vector<LaneWidth> laneWidths;
//            double lane_width = 0.0;
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
//                lane_width = (a +
//                              b * sOffset_ +
//                              c * pow(sOffset_,2) +
//                              d * pow(sOffset_,3));
//
//                // split curve segment by sOffset
//                CurveSegment lane_curve_segment;
//                PointENU start_point;
//                start_point = curveSegment.start_position;
//                RoadSectionInternal road_section_internal;
//                lane_curve_segment.lineSegment    = curveSegment.lineSegment;
//                lane_curve_segment.s              = curveSegment.s;
//                lane_curve_segment.length         = curveSegment.length;
//                lane_curve_segment.curveType        = curveSegment.curveType;
//                lane_curve_segment.laneOffset     = sOffset_;
//                lane_curve_segment.start_position = start_point;
//                lane_curve_segment.heading        = start_point.hdg;
//
//                if (sOffset_ >= lane_curve_segment.s){
//                    ParseLane(*lane_node, &lane);
//                    // compute right lane boundary
//                    ParseLeftCurve(&lane,lane_width,center_lane_internal.lane.central_curve.segment.at(0));
//                    break;
//                }else {
//                    width_node = width_node->NextSiblingElement("width");
//                }
//            }
//            lane.lane_widths = laneWidths;
//            lane_internal.lane = lane;
//            lane_internal.isCenter = false;
//            lanes->push_back(lane_internal);
//            lane_node = lane_node->NextSiblingElement("lane");
//        }
//    }


//    // right
//    const tinyxml2::XMLElement* right_node = xml_node.FirstChildElement("right");
//    if (right_node) {
//        LaneInternal lane_internal;
//        const tinyxml2::XMLElement* lane_node = right_node->FirstChildElement("lane");
//        while (lane_node) {
//            MpLane lane;
//            const tinyxml2::XMLElement* width_node = lane_node->FirstChildElement("width");
//            std::vector<LaneWidth> laneWidths;
//            double lane_width = 0.0;
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
//                lane_width = (a +
//                               b * sOffset_ +
//                               c * pow(sOffset_,2) +
//                               d * pow(sOffset_,3));
//
//                // split curve segment by sOffset
//                CurveSegment lane_curve_segment;
//                PointENU start_point;
//                start_point = curveSegment.start_position;
//                RoadSectionInternal road_section_internal;
//                lane_curve_segment.lineSegment    = curveSegment.lineSegment;
//                lane_curve_segment.s              = curveSegment.s;
//                lane_curve_segment.length         = curveSegment.length;
//                lane_curve_segment.curveType        = curveSegment.curveType;
//                lane_curve_segment.laneOffset     = sOffset_;
//                lane_curve_segment.start_position = start_point;
//                lane_curve_segment.heading        = start_point.hdg;
//
//                if (sOffset_ >= lane_curve_segment.s){
//                    ParseLane(*lane_node, &lane);
//                    // compute right lane boundary
//                    ParseRightCurve(&lane,lane_width,center_lane_internal.lane.central_curve.segment.at(0));
//                    break;
//                }else {
//                    width_node = width_node->NextSiblingElement("width");
//                }
//            }
//            lane.lane_widths = laneWidths;
//            lane_internal.lane = lane;
//            lane_internal.isCenter = false;
//            lanes->push_back(lane_internal);
//            lane_node = lane_node->NextSiblingElement("lane");
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
//    int success = ToPbLaneType(lane_type, &e_lane_type);
//    if (!success) {
//        std::string err_msg = "Error convert lane type to pb lane type.";
//        return 0;
//    }
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
int LaneXmlParser::ParseLeftCurve(MpLane* lane,
                                  double d_offset,
                                  CurveSegment center_curveSegment) {

    LOG(ERROR) << "------------------Left line START ------------------";
    CurveSegment res_curveSegment;
    double delta_s = 0.2;
    PointENU start_point = center_curveSegment.start_position;
    PointENU lane_start_point;
    double s = center_curveSegment.s;
    double x = center_curveSegment.start_position.x;
    double y = center_curveSegment.start_position.y;
    double hdg = center_curveSegment.start_position.hdg;
    double cos_hdg = cos(hdg);
    double sin_hdg = sin(hdg);
    double theta = 0;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    if (center_curveSegment.curveType == entity::CurveSegment::CurveType_Line){
        double internal_start_x = x - d_offset * cos_theta;
        double internal_start_y = y + d_offset * sin_theta;
        lane_start_point.x = internal_start_x;
        lane_start_point.y = internal_start_y;
        lane_start_point.s = start_point.s;
        lane_start_point.hdg = hdg;
        res_curveSegment.start_position = lane_start_point;
        for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
            double dx = lane_start_point.x + (delta_s * i) * cos_hdg;
            double dy = lane_start_point.y + (delta_s * i) * sin_hdg;
            PointENU point(dx,dy,0,s,center_curveSegment.heading);
            s += delta_s;
            LOG(ERROR) << dx << "," << dy;
            res_curveSegment.lineSegment.points.push_back(point);
        }
    }else if (center_curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
        double internal_start_x = x - d_offset * cos_theta;
        double internal_start_y = y + d_offset * sin_theta;
        lane_start_point.x = internal_start_x;
        lane_start_point.y = internal_start_y;
        lane_start_point.s = start_point.s;
        lane_start_point.hdg = hdg;
        double curvature = start_point.curveture;
        double radius = 1 / curvature;
        res_curveSegment.start_position = lane_start_point;
        for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
            const double ref_line_ds = delta_s * i;
            const double angle_at_s = ref_line_ds * curvature - M_PI / 2;
            const double xd = radius * (cos(hdg + angle_at_s) - sin_hdg) + internal_start_x;
            const double yd = radius * (sin(hdg + angle_at_s) + cos_hdg) + internal_start_y;
            const double tangent = hdg + delta_s * curvature;
            PointENU* pointEnu = new PointENU(xd,yd,0,s,tangent);
            s += delta_s;
            LOG(ERROR) << xd << "," << yd;
            pointEnu->curveture = curvature;
            res_curveSegment.lineSegment.points.push_back(*pointEnu);
        }
    }
    lane->central_curve.segment.push_back(center_curveSegment);
    lane->left_boundary.curve.segment.push_back(res_curveSegment);
    LOG(ERROR) << "------------------Left LINE END ------------------";
    return 1;
}

int LaneXmlParser::ParseRightCurve(MpLane* lane,
                                  double d_offset,
                                  CurveSegment center_curveSegment) {

    LOG(ERROR) << "------------------Right line START ------------------";
    CurveSegment res_curveSegment;
    double delta_s = 0.2;
    PointENU start_point = center_curveSegment.start_position;
    PointENU lane_start_point;
    double s = center_curveSegment.s;
    double x = center_curveSegment.start_position.x;
    double y = center_curveSegment.start_position.y;
    double hdg = center_curveSegment.start_position.hdg;
    double cos_hdg = cos(hdg);
    double sin_hdg = sin(hdg);
    double theta = M_PI;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    if (center_curveSegment.curveType == entity::CurveSegment::CurveType_Line){
        double internal_start_x = x + d_offset * cos_theta;
        double internal_start_y = y - d_offset * sin_theta;
        lane_start_point.x = internal_start_x;
        lane_start_point.y = internal_start_y;
        lane_start_point.s = start_point.s;
        lane_start_point.hdg = hdg;
        res_curveSegment.start_position = lane_start_point;
        for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
            double dx = lane_start_point.x + (delta_s * i) * cos_hdg;
            double dy = lane_start_point.y + (delta_s * i) * sin_hdg;
            PointENU point(dx,dy,0,s,center_curveSegment.heading);
            s += delta_s;
            LOG(ERROR) << dx << "," << dy;
            res_curveSegment.lineSegment.points.push_back(point);
        }
    }else if (center_curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
        double internal_start_x = x + d_offset * cos_theta;
        double internal_start_y = y - d_offset * sin_theta;
        lane_start_point.x = internal_start_x;
        lane_start_point.y = internal_start_y;
        lane_start_point.s = start_point.s;
        lane_start_point.hdg = hdg;
        double curvature = start_point.curveture;
        double radius = 1 / curvature;
        res_curveSegment.start_position = lane_start_point;
        for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
            const double ref_line_ds = delta_s * i;
            const double angle_at_s = ref_line_ds * curvature - M_PI / 2;
            const double xd = radius * (cos(hdg + angle_at_s) - sin_hdg) + internal_start_x;
            const double yd = radius * (sin(hdg + angle_at_s) + cos_hdg) + internal_start_y;
            const double tangent = hdg + delta_s * curvature;
            PointENU* pointEnu = new PointENU(xd,yd,0,s,tangent);
            s += delta_s;
            LOG(ERROR) << xd << "," << yd;
            pointEnu->curveture = curvature;
            res_curveSegment.lineSegment.points.push_back(*pointEnu);
        }
    }
    lane->central_curve.segment.push_back(center_curveSegment);
    lane->right_boundary.curve.segment.push_back(res_curveSegment);
    LOG(ERROR) << "------------------RIGHT LINE END ------------------";
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
    CurveSegment curveSegment_;
    LaneInternal laneInternal_;
    for (size_t i = 0; i < lastRoadSectionInternal.lanes.size(); ++i) {
        if (lastRoadSectionInternal.lanes.at(i).isCenter){
            laneInternal_ = lastRoadSectionInternal.lanes.at(i);
            break;
        }
    }
    PointENU start_point = curveSegment.start_position;
    if (isFirstSection){
        curveSegment_ = curveSegment;
    }else{
        curveSegment_ = laneInternal_.lane.central_curve.segment.at(0);
    }
    double delta_s = 0.2;
    PointENU center_start_point;
    double s = curveSegment_.s;
    double x = curveSegment_.start_position.x;
    double y = curveSegment_.start_position.y;
    double hdg = curveSegment_.start_position.hdg;
    double cos_hdg = cos(hdg);
    double sin_hdg = sin(hdg);
    double theta = 0;
    if (x < 0 && y < 0 && hdg < 0){
        theta =  0;
    }else if (x < 0 && y < 0 && hdg > 0){
        theta =  M_PI + M_PI_2;
    }
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
//    LOG(ERROR) << "--------------------- CENTER LINE START---------------------";
    if (curveSegment.curveType == entity::CurveSegment::CurveType_Line){
        double internal_start_x = 0;
        double internal_start_y = 0;
        if (isFirstSection){
            internal_start_x = x + cos_theta * d_offset;
            internal_start_y = y + sin_theta * d_offset;
        }else{
            internal_start_x = x + cos_theta;
            internal_start_y = y + sin_theta;
        }

        center_start_point.x = internal_start_x;
        center_start_point.y = internal_start_y;
        center_start_point.s = start_point.s;
        center_start_point.hdg = hdg;
        center_start_point.curveture = 0;
        size_t lineSegment_size =curveSegment.lineSegment.points.size();
        for (size_t i = 0; i < lineSegment_size; ++i) {
            double dx = center_start_point.x + (delta_s * i) * cos_hdg;
            double dy = center_start_point.y + (delta_s * i) * sin_hdg;
            PointENU point(dx,dy,0,s,curveSegment_.heading);
            s += delta_s;
            res_curveSegment.lineSegment.points.push_back(point);
            LOG(ERROR) << dx << "," << dy;
//            LOG(ERROR) << "LINE";
        }
        res_curveSegment.s = res_curveSegment.lineSegment.points[lineSegment_size-1].s;
        res_curveSegment.start_position = res_curveSegment.lineSegment.points[lineSegment_size-1];
        res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
        res_curveSegment.end_position = res_curveSegment.lineSegment.points[lineSegment_size-1];
        res_curveSegment.heading = res_curveSegment.lineSegment.points[lineSegment_size-1].hdg;
    }else if (curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
        double curvature = start_point.curveture;
        center_start_point.curveture = curvature;
        double radius = 1 / curvature;
        size_t lineSegment_size =curveSegment.lineSegment.points.size();
        for (size_t i = 0; i < lineSegment_size; ++i) {
            const double ref_line_ds = delta_s * i;
            const double angle_at_s = ref_line_ds * curvature - M_PI / 2;
            const double xd = radius * (cos(hdg + angle_at_s) - sin_hdg) + x;
            const double yd = radius * (sin(hdg + angle_at_s) + cos_hdg) + y;
            const double tangent = hdg + ref_line_ds * curvature;
            PointENU* pointEnu = new PointENU(xd,yd,0,s,tangent);
            s += delta_s;
            pointEnu->curveture = curvature;
            res_curveSegment.lineSegment.points.push_back(*pointEnu);
            LOG(ERROR) << xd << "," << yd;
//            LOG(ERROR) << "ARC";
        }
        res_curveSegment.s = res_curveSegment.lineSegment.points[lineSegment_size-1].s;
        res_curveSegment.start_position = res_curveSegment.lineSegment.points[lineSegment_size-1];
        res_curveSegment.curveType = entity::CurveSegment::CurveType_ARC;
        res_curveSegment.end_position = res_curveSegment.lineSegment.points[lineSegment_size-1];
        res_curveSegment.heading = res_curveSegment.lineSegment.points[lineSegment_size-1].hdg;
    }
//    LOG(ERROR) << "--------------------- CENTER LINE END---------------------";
    laneInternal->lane.central_curve.segment.push_back(res_curveSegment);
    return 1;
}

}
}
}
