//
// Created by ai on 23-3-7.
//

#include "lane_xml_parser.h"
#include <iostream>
#define print_center
//#define print_left
//#define print_right

#define SIN_270_ sin(3*M_PI_2)

const double Epsilon = 0.000001;

constexpr double kDuplicatedPointsEpsilon = 1e-7;
const double SIN_360 = std::sin(3*M_PI_2);
const double SIN_270 = std::sin(2*M_PI);

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
  std::vector<MpLaneOffSet> lane_offset_vec;
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
    lane_offset.d_offset = d_offset;
    lane_offset_vec.push_back(lane_offset);
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
    laneOffset_node = laneOffset_node->NextSiblingElement("laneOffset");
  }
  size_t offset_idx = 0;
  for (size_t i = 0; i < lane_offset_vec.size(); ++i) {
    MpLaneOffSet laneOffSet = lane_offset_vec.at(i);
    if (i == lane_offset_vec.size()){
      offset_idx = i;
      break;
    }
    if (curveSegment.s >= laneOffSet.s)
      continue;
    offset_idx = i - 1;
    break;
  }
  std::vector<LaneInternal> lanes;
  const tinyxml2::XMLElement* laneSection_node = lanes_node->FirstChildElement("laneSection");
  ParseLaneSection(*laneSection_node,&lanes,d_offset,curveSegment);
//            road_section_internal.lanes = lanes;
  roadSection->lanes = lanes;

  return 1;
}
bool compare1(LaneInternal l1,LaneInternal l2){
  if (std::stoi(l1.lane.id) > std::stoi(l2.lane.id)){
    return true;
  }else{
    return false;
  }
}

bool compare2(LaneInternal l1,LaneInternal l2){
  if (std::stoi(l1.lane.id) < std::stoi(l2.lane.id)){
    return true;
  }else{
    return false;
  }
}

int LaneXmlParser::ParseLaneSection(const tinyxml2::XMLElement &xml_node,
                                    std::vector<LaneInternal> *lanes,
                                    double d_offset,
                                    CurveSegment curveSegment) {
  CHECK_NOTNULL(lanes);

  // center
  const tinyxml2::XMLElement* center_node = xml_node.FirstChildElement("center");
  LaneInternal center_lane_internal;
  std::vector<LaneInternal> lanes_center;
  if (center_node) {
    const tinyxml2::XMLElement* lane_node = center_node->FirstChildElement("lane");
    if (lane_node) {
      ParseCenterLane(*lane_node, &center_lane_internal);
      // center curve
      std::shared_ptr<MpLane> lane_ptr_(new MpLane());
//            ParseCenterCurve(&center_lane_internal,d_offset,curveSegment);
      ParseLeftCenterCurve(&center_lane_internal,d_offset,curveSegment);
    }
    center_lane_internal.isCenter = true;
    lanes_center.push_back(center_lane_internal);
  }

//     left
  const tinyxml2::XMLElement* left_node = xml_node.FirstChildElement("left");
  std::vector<LaneInternal> lanes_left;
  if (left_node) {
    LaneInternal lane_internal;
    double lane_width = 0.0;
    const tinyxml2::XMLElement* lane_node = left_node->FirstChildElement("lane");
    while (lane_node) {
      MpLane lane;
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
        width.sOffset  = sOffset_;
        width.a = a;
        width.b = b;
        width.c = c;
        width.d = d;
        if (isCorner){
          lane_width = (a +
              b * sOffset_ +
              c * pow(sOffset_,2) +
              d * pow(sOffset_,3));
        }else{
          lane_width += (a +
              b * sOffset_ +
              c * pow(sOffset_,2) +
              d * pow(sOffset_,3));
        }

        // split curve segment by sOffset
        CurveSegment lane_curve_segment;
        PointENU start_point;
        start_point = curveSegment.start_position;
        RoadSectionInternal road_section_internal;
        lane_curve_segment.lineSegment    = curveSegment.lineSegment;
        lane_curve_segment.s              = curveSegment.s;
        lane_curve_segment.length         = curveSegment.length;
        lane_curve_segment.curveType        = curveSegment.curveType;
        lane_curve_segment.laneOffset     = sOffset_;
        lane_curve_segment.start_position = start_point;
        lane_curve_segment.heading        = start_point.hdg;

//                if (sOffset_ >= lane_curve_segment.s){
        ParseLane(*lane_node, &lane);

        width.lane_id = lane.id;
        laneWidths.push_back(width);
//                    // compute right lane boundary
//                    ParseLeftCurve(&lane,lane_width,center_lane_internal.lane.central_curve.segment.at(0));
//                    break;
//                }else {
//                    width_node = width_node->NextSiblingElement("width");
//                }
        width_node = width_node->NextSiblingElement("width");
      }
      lane.lane_widths = laneWidths;
      lane_internal.lane = lane;
      lane_internal.isCenter = false;
      lanes_left.push_back(lane_internal);
      lane_node = lane_node->NextSiblingElement("lane");
    }
  }
  double calc_left_width = 0.0;
  double left_width = 0.0;
  std::sort(lanes_left.begin(),lanes_left.end(), compare2);
  for (size_t i = 0; i < lanes_left.size(); ++i) {
    LaneInternal lane_left_internal = lanes_left.at(i);
    Lane lane;

    size_t idx_width = 0;
    for (size_t j = 0; j < lane_left_internal.lane.lane_widths.size(); ++j) {
      LaneWidth lane_width = lane_left_internal.lane.lane_widths.at(j);
      if ( j == lane_left_internal.lane.lane_widths.size() -1 ){
        idx_width = j;
        break;
      }
      if (curveSegment.s >= lane_width.sOffset)
        continue;
      idx_width = j - 1;
      break;
    }
    LaneWidth lane_width = lane_left_internal.lane.lane_widths.at(idx_width);
    double sOffset = lane_width.sOffset;
    double ds = curveSegment.s + sOffset;
    calc_left_width += (lane_width.a + lane_width.b * ds + lane_width.c * pow(ds,2) + lane_width.d * pow(ds,3));

    ParseLeftCurve(&lane_left_internal.lane,calc_left_width,center_lane_internal.lane.central_curve.segment.at(0));
  }


  // right
  const tinyxml2::XMLElement* right_node = xml_node.FirstChildElement("right");
  std::vector<LaneInternal> lanes_right;
  if (right_node) {
    LaneInternal lane_internal;
    const tinyxml2::XMLElement* lane_node = right_node->FirstChildElement("lane");
    while (lane_node) {
      MpLane lane;
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
        width.sOffset  = sOffset_;
        width.a = a;
        width.b = b;
        width.c = c;
        width.d = d;
        laneWidths.push_back(width);

        // split curve segment by sOffset
        CurveSegment lane_curve_segment;
        PointENU start_point;
        start_point = curveSegment.start_position;
        RoadSectionInternal road_section_internal;
        lane_curve_segment.lineSegment    = curveSegment.lineSegment;
        lane_curve_segment.s              = curveSegment.s;
        lane_curve_segment.length         = curveSegment.length;
        lane_curve_segment.curveType        = curveSegment.curveType;
        lane_curve_segment.laneOffset     = sOffset_;
        lane_curve_segment.start_position = start_point;
        lane_curve_segment.heading        = start_point.hdg;

//                if (sOffset_ >= lane_curve_segment.s){
        ParseLane(*lane_node, &lane);

        width.lane_id = lane.id;
        laneWidths.push_back(width);
        width_node = width_node->NextSiblingElement("width");
      }
      lane.lane_widths = laneWidths;
      lane_internal.lane = lane;
      lane_internal.isCenter = false;
      lanes_right.push_back(lane_internal);
      lane_node = lane_node->NextSiblingElement("lane");
    }
  }
  double calc_right_width = 0.0;
  std::sort(lanes_right.begin(),lanes_right.end(), compare1);
  for (size_t i = 0; i < lanes_right.size(); ++i) {
    LaneInternal lane_right_internal = lanes_right.at(i);
    Lane lane;

    size_t idx_width = 0;
    for (size_t j = 0; j < lane_right_internal.lane.lane_widths.size(); ++j) {
      LaneWidth lane_width = lane_right_internal.lane.lane_widths.at(j);
      if ( j == lane_right_internal.lane.lane_widths.size() -1 ){
        idx_width = j;
        break;
      }
      if (curveSegment.s >= lane_width.sOffset)
        continue;
      idx_width = j - 1;
      break;
    }
    LaneWidth lane_width = lane_right_internal.lane.lane_widths.at(idx_width);
    double sOffset = lane_width.sOffset;
    double ds = curveSegment.s + sOffset;
    calc_right_width += lane_width.a + lane_width.b * ds + lane_width.c * pow(ds,2) + lane_width.d * pow(ds,3);

    ParseRightCurve(&lane_right_internal.lane,calc_right_width,center_lane_internal.lane.central_curve.segment.at(0));
  }

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

//    LOG(ERROR) << "------------------Left line START ------------------";
  CurveSegment res_curveSegment;
  double delta_s = 0.2;
  PointENU start_point = center_curveSegment.start_position;
  double s = center_curveSegment.s;
  double hdg = center_curveSegment.start_position.hdg;
  double theta = 0;
  if (center_curveSegment.curveType == entity::CurveSegment::CurveType_Line){
    PointENU first_point;
    for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = center_curveSegment.lineSegment.points.at(i);
      double x = point.x;
      double y = point.y;

      double k = 0;
      if (i == 0 ) {
        continue;
      }else{
        PointENU point2 = center_curveSegment.lineSegment.points.at(i-1);
        double last_x = point2.x;
        double last_y = point2.y;
        k = (y - last_y)/(x - last_x);
      }
      double b = y-k*x;
      double x_ = 0.0;
      // Calculate the coordinates on the vertical line
      double alpha =  0;
      double beta =  0;
      double gama =  0;
      alpha = 1+(1/ pow(k,2));
      beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
      gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
          pow(d_offset,2);
      math::CommonMath commonMath;
      std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);

      if ( 0 < center_curveSegment.heading < M_PI/2 || M_PI < center_curveSegment.heading < 3*M_PI/2|| -M_PI < center_curveSegment.heading < -M_PI/2){
        x_ = std::max(std::get<1>(res),std::get<0>(res));
      } else if (-M_PI/2 < center_curveSegment.heading < 0 || M_PI/2 < center_curveSegment.heading < M_PI || -3*M_PI/2 < center_curveSegment.heading < -M_PI){
        x_ = std::min(std::get<1>(res),std::get<0>(res));
      }

      const double xd = x_;
      const double yd = -1/k*xd+(k+1/k)*x+b;
      PointENU pointEnu(xd,yd,0,s,point.hdg);
      s += delta_s;
#ifdef print_left
      LOG(ERROR) << xd << "," << yd<< "," << point.hdg;
#endif
      res_curveSegment.lineSegment.points.push_back(pointEnu);
      if (i == 1){
        first_point = pointEnu;
      }
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
    res_curveSegment.heading = first_point.hdg;
  }else if (center_curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
    PointENU first_point;
    double curvature = start_point.curveture;
    for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = center_curveSegment.lineSegment.points.at(i);
      double x = point.x;
      double y = point.y;
      double k = 0;
      if (i == 0 ) {
        continue;
      }else{
        PointENU point2 = center_curveSegment.lineSegment.points.at(i-1);
        double last_x = point2.x;
        double last_y = point2.y;
        k = (y - last_y)/(x - last_x);
      }
      double b = y-k*x;
      double x_ = 0.0;
      // Calculate the coordinates on the vertical line
      double alpha =  0;
      double beta =  0;
      double gama =  0;
      alpha = 1+(1/ pow(k,2));
      beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
      gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
          pow(d_offset,2);
      math::CommonMath commonMath;
      std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);

//      if (isCorner){
//        if (alpha > 1 && alpha < 2){
//          x_ = std::min(std::get<1>(res),std::get<0>(res));
//        } else {
//          x_ = std::max(std::get<1>(res),std::get<0>(res));
//        }
//      }else{
//        if (alpha > 1 && alpha < 2  ) {
//          x_ = std::min(std::get<1>(res),std::get<0>(res));
//        } else if(SIN_270 - sin(center_curveSegment.heading) < Epsilon) {
//          x_ = std::min(std::get<1>(res),std::get<0>(res));
//        }else{
//          x_ = std::max(std::get<1>(res),std::get<0>(res));
//        }
//      }
      if ( 0 < center_curveSegment.heading < M_PI/2 || M_PI < center_curveSegment.heading < 3*M_PI/2|| -M_PI < center_curveSegment.heading < -M_PI/2){
        x_ = std::max(std::get<1>(res),std::get<0>(res));
      } else if (-M_PI/2 < center_curveSegment.heading < 0 || M_PI/2 < center_curveSegment.heading < M_PI || -3*M_PI/2 < center_curveSegment.heading < -M_PI){
        x_ = std::min(std::get<1>(res),std::get<0>(res));
      }
      const double xd = x_;
      const double yd = -1/k*xd+(k+1/k)*x+b;
      const double tangent = hdg + delta_s * curvature;
      PointENU pointEnu(xd,yd,0,s,point.hdg);
      s += delta_s;
#ifdef print_left
      LOG(ERROR) << xd << "," << yd<< "," << point.hdg;
#endif
      pointEnu.curveture = curvature;
      res_curveSegment.lineSegment.points.push_back(pointEnu);
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
    res_curveSegment.heading = first_point.hdg;
  }
  lane->central_curve.segment.push_back(center_curveSegment);
  lane->left_boundary.curve.segment.push_back(res_curveSegment);
//    LOG(ERROR) << "------------------Left LINE END ------------------";
  return 1;
}

int LaneXmlParser::ParseRightCurve(MpLane* lane,
                                   double d_offset,
                                   CurveSegment center_curveSegment) {

//    LOG(ERROR) << "------------------Right line START ------------------";
  CurveSegment res_curveSegment;
  double delta_s = 0.2;
  PointENU start_point = center_curveSegment.start_position;
  double s = center_curveSegment.s;
  double hdg = center_curveSegment.start_position.hdg;
  if (center_curveSegment.curveType == entity::CurveSegment::CurveType_Line){
    PointENU first_point;
    for (size_t i = 0; i < center_curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = center_curveSegment.lineSegment.points.at(i);
      double x = point.x;
      double y = point.y;

      double k = 0;
      if (i == 0 ) {
        continue;
      }else{
        PointENU point2 = center_curveSegment.lineSegment.points.at(i-1);
        double last_x = point2.x;
        double last_y = point2.y;
        k = (y - last_y)/(x - last_x);
      }
      double b = y-k*x;
      double x_ = 0.0;
      // Calculate the coordinates on the vertical line
      double alpha =  0;
      double beta =  0;
      double gama =  0;
      alpha = 1+(1/ pow(k,2));
      beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
      gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
          pow(d_offset,2);
      math::CommonMath commonMath;
      std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);
//      if (isCorner){
//        if (alpha > 1 && alpha < 2){
//          x_ = std::min(std::get<1>(res),std::get<0>(res));
//        } else {
//          x_ = std::max(std::get<1>(res),std::get<0>(res));
//        }
//      }else{
//        if (alpha > 1 && alpha < 2){
//          x_ = std::max(std::get<1>(res),std::get<0>(res));
//        } else {
//          x_ = std::min(std::get<1>(res),std::get<0>(res));
//        }
//      }

      if ( 0 < center_curveSegment.heading < M_PI/2 || M_PI < center_curveSegment.heading < 3*M_PI/2|| -M_PI < center_curveSegment.heading < -M_PI/2){
        x_ = std::max(std::get<1>(res),std::get<0>(res));
      } else if (-M_PI/2 < center_curveSegment.heading < 0 || M_PI/2 < center_curveSegment.heading < M_PI || -3*M_PI/2 < center_curveSegment.heading < -M_PI){
        x_ = std::min(std::get<1>(res),std::get<0>(res));
      }
      const double xd = x_;
      const double yd = -1/k*xd+(k+1/k)*x+b;
      PointENU pointEnu(xd,yd,0,s,point.hdg);
      s += delta_s;
#ifdef print_right
      LOG(ERROR) << xd << "," << yd<<","<<point.hdg;
#endif
      res_curveSegment.lineSegment.points.push_back(pointEnu);
      if (i == 1){
        first_point = pointEnu;
      }
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
    res_curveSegment.heading = first_point.hdg;
  }else if (center_curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
    PointENU first_point;
    double curvature = start_point.curveture;
    for (size_t i = 0; i < center_curveSegment.lineSegment.points.size();  ++i) {
      PointENU point = center_curveSegment.lineSegment.points.at(i);
      double x = point.x;
      double y = point.y;
      double k = 0;
      if (i == 0 ) {
        continue;
      }else{
        PointENU point2 = center_curveSegment.lineSegment.points.at(i-1);
        double last_x = point2.x;
        double last_y = point2.y;
        k = (y - last_y)/(x - last_x);
      }
      double b = y-k*x;
      double x_ = 0.0;
      // Calculate the coordinates on the vertical line
      double alpha =  0;
      double beta =  0;
      double gama =  0;
      alpha = 1+(1/ pow(k,2));
      beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
      gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
          pow(d_offset,2);
      math::CommonMath commonMath;
      std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);
//      if ( 0 < center_curveSegment.heading < M_PI/2 || M_PI < center_curveSegment.heading < 3*M_PI/2|| -M_PI < center_curveSegment.heading < -M_PI/2){
//        x_ = std::max(std::get<1>(res),std::get<0>(res));
//      } else if (-M_PI/2 < center_curveSegment.heading < 0 || M_PI/2 < center_curveSegment.heading < M_PI || -3*M_PI/2 < center_curveSegment.heading < -M_PI){
//        x_ = std::min(std::get<1>(res),std::get<0>(res));
//      }
//      if ( 0 < center_curveSegment.heading - M_PI < M_PI/2){
//        x_ = std::max(std::get<1>(res),std::get<0>(res));
//      } else if (-M_PI/2 < center_curveSegment.heading < 0 || M_PI/2 < center_curveSegment.heading < M_PI || -3*M_PI/2 < center_curveSegment.heading < -M_PI){
//        x_ = std::min(std::get<1>(res),std::get<0>(res));
//      }
      const double x_max = std::max(std::get<1>(res),std::get<0>(res));
      const double x_min = std::min(std::get<1>(res),std::get<0>(res));
      const double y_max = -1/k*x_max+(k+1/k)*x+b;
      const double y_min = -1/k*x_min+(k+1/k)*x+b;
      const double xd = x_;
      const double yd = -1/k*xd+(k+1/k)*x+b;
      const double tangent = hdg + delta_s * curvature;
      PointENU pointEnu(xd,yd,0,s,point.hdg);
      s += delta_s;
#ifdef print_right
      LOG(ERROR) << xd << "," << yd<<","<<point.hdg;
#endif
      pointEnu.curveture = curvature;
      res_curveSegment.lineSegment.points.push_back(pointEnu);
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
    res_curveSegment.heading = first_point.hdg;

  }
  lane->central_curve.segment.push_back(center_curveSegment);
  lane->right_boundary.curve.segment.push_back(res_curveSegment);
//    LOG(ERROR) << "------------------RIGHT LINE END ------------------";
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
  curveSegment_ = curveSegment;
  double delta_s = 0.2;
  double s = curveSegment_.s;
  double hdg = curveSegment_.start_position.hdg;
//    LOG(ERROR) << "--------------------- CENTER LINE START---------------------";
  if (curveSegment.curveType == entity::CurveSegment::CurveType_Line){
    PointENU first_point;
    for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = curveSegment.lineSegment.points.at(i);
      double x = point.x;
      double y = point.y;

      double k = 0;
      if (i == 0 ) {
        continue;
      }else{
        PointENU point2 = curveSegment.lineSegment.points.at(i-1);
        double last_x = point2.x;
        double last_y = point2.y;
        k = (y - last_y)/(x - last_x);
      }
      double b = y-k*x;
      double x_ = 0.0;
      // Calculate the coordinates on the vertical line
      double alpha =  0;
      double beta =  0;
      double gama =  0;
      alpha = 1+(1/ pow(k,2));
      beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
      gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
          pow(d_offset,2);
      math::CommonMath commonMath;
      std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);
      if (alpha > 1 && alpha < 2){
        x_ = std::max(std::get<1>(res),std::get<0>(res));
      } else {
        x_ = std::min(std::get<1>(res),std::get<0>(res));
      }
      const double xd = x_;
      const double yd = -1/k*xd+(k+1/k)*x+b;
      PointENU pointEnu(xd,yd,0,s,point.hdg);
      s += delta_s;
#ifdef print_center
      LOG(ERROR) << xd << "," << yd <<","<<point.hdg;
#endif
      res_curveSegment.lineSegment.points.push_back(pointEnu);
      if (i == 1){
        first_point = pointEnu;
      }
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
    res_curveSegment.heading = first_point.hdg;
  }else if (curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
    PointENU first_point;
    double curvature = start_point.curveture;
    for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = curveSegment.lineSegment.points.at(i);
      double x = point.x;
      double y = point.y;
      double k = 0;
      if (i == 0 ) {
        continue;
      }else{
        PointENU point2 = curveSegment.lineSegment.points.at(i-1);
        double last_x = point2.x;
        double last_y = point2.y;
        k = (y - last_y)/(x - last_x);
      }
      double b = y-k*x;
      double x_ = 0.0;
      // Calculate the coordinates on the vertical line
      double alpha =  0;
      double beta =  0;
      double gama =  0;
      alpha = 1+(1/ pow(k,2));
      beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
      gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
          pow(d_offset,2);
      math::CommonMath commonMath;
      std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);
      x_ = std::max(std::get<1>(res),std::get<0>(res));
      const double xd = x_;
      const double yd = -1/k*xd+(k+1/k)*x+b;
      const double tangent = hdg + delta_s * curvature;
      PointENU pointEnu(xd,yd,0,s,tangent);
      s += delta_s;
#ifdef print_center
      LOG(ERROR) << xd << "," << yd<<","<<tangent;
#endif
      pointEnu.curveture = curvature;
      res_curveSegment.lineSegment.points.push_back(pointEnu);
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_ARC;
    res_curveSegment.heading = first_point.hdg;
  }
//    LOG(ERROR) << "--------------------- CENTER LINE END---------------------";
  laneInternal->lane.central_curve.segment.push_back(res_curveSegment);
  return 1;
}



int LaneXmlParser::ParseLeftCenterCurve(LaneInternal* laneInternal,
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
  curveSegment_ = curveSegment;
  double delta_s = 0.2;
  double s = curveSegment_.s;
  double hdg = curveSegment_.start_position.hdg;
//    LOG(ERROR) << "--------------------- CENTER LINE START---------------------";
  if (curveSegment.curveType == entity::CurveSegment::CurveType_Line){
    PointENU first_point;
    for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = curveSegment.lineSegment.points.at(i);

      if (abs(kDuplicatedPointsEpsilon) > abs(d_offset)){
        PointENU pointEnu(point.x,point.y,0,point.s,point.hdg);
#ifdef print_center
        LOG(ERROR) << point.x << "," << point.y<<","<<point.hdg;
#endif
        res_curveSegment.lineSegment.points.push_back(pointEnu);
      }else{
        double x = point.x;
        double y = point.y;

        double k = 0;
        if (i == 0 ) {
          continue;
        }else{
          PointENU point2 = curveSegment.lineSegment.points.at(i-1);
          double last_x = point2.x;
          double last_y = point2.y;
          k = (y - last_y)/(x - last_x);
        }
        double b = y-k*x;
        double x_ = 0.0;
        // Calculate the coordinates on the vertical line
        double alpha =  0;
        double beta =  0;
        double gama =  0;
        alpha = 1+(1/ pow(k,2));
        beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
        gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
            pow(d_offset,2);
        math::CommonMath commonMath;
        std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);
//        if (alpha > 1 && alpha < 2){
//          x_ = std::max(std::get<1>(res),std::get<0>(res));
//        } else {
//          x_ = std::min(std::get<1>(res),std::get<0>(res));
//        }

        PointENU res_point;
        if (M_PI/2 < curveSegment.heading < M_PI){

        }

//
//        const double x_min = std::min(std::get<1>(res),std::get<0>(res));
////        const double xd = x_;
//        const double y_max = -1/k*x_max+(k+1/k)*x+b;
//        const double y_min = -1/k*x_min+(k+1/k)*x+b;
//
//        PointENU pointEnu_max(x_max,y_max,0,s,point.hdg);
//        PointENU pointEnu_min(x_min,y_min,0,s,point.hdg);
//        PointENU temp_point_1;
//        PointENU temp_point_2 = point;
//        if (i == 0){
//          temp_point_1 = start_point;
//        }else{
//          if (i+1 < curveSegment.lineSegment.points.size()){
//            temp_point_1 = curveSegment.lineSegment.points.at(i+1);
//          } else{
//            temp_point_1 = curveSegment.lineSegment.points.at(i-1);
//            temp_point_2 = curveSegment.lineSegment.points.at(i);
//          }
//        }
//        if (commonMath.checkPosePointLeftOrRight(temp_point_1,temp_point_2,pointEnu_max)){
//          LOG(ERROR) << " left ";
//          res_point = pointEnu_max;
//        }else{
//          LOG(ERROR) << " right";
//          res_point = pointEnu_min;
//        }
        s += delta_s;
#ifdef print_center
        LOG(ERROR) << res_point.x << "," << res_point.y<<","<<point.hdg;
#endif
        res_curveSegment.lineSegment.points.push_back(res_point);
        if (i == 1){
          first_point = res_point;
        }
      }
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_Line;
    res_curveSegment.heading = first_point.hdg;
  }else if (curveSegment.curveType == entity::CurveSegment::CurveType_ARC){
    PointENU first_point;
    double curvature = start_point.curveture;
    for (size_t i = 0; i < curveSegment.lineSegment.points.size(); ++i) {
      PointENU point = curveSegment.lineSegment.points.at(i);
      if (kDuplicatedPointsEpsilon > d_offset){
        PointENU pointEnu(point.x,point.y,0,point.s,point.hdg);
        res_curveSegment.lineSegment.points.push_back(pointEnu);
#ifdef print_center
        LOG(ERROR) << point.x << "," << point.y<<","<<point.hdg;
#endif
      }else{
        double x = point.x;
        double y = point.y;
        double k = 0;
        if (i == 0 ) {
          continue;
        }else{
          PointENU point2 = curveSegment.lineSegment.points.at(i-1);
          double last_x = point2.x;
          double last_y = point2.y;
          k = (y - last_y)/(x - last_x);
        }
        double b = y-k*x;
        double x_ = 0.0;
        // Calculate the coordinates on the vertical line
        double alpha =  0;
        double beta =  0;
        double gama =  0;
        alpha = 1+(1/ pow(k,2));
        beta = (2*y/k-2*x)+(-2*x-2*1/pow(k,2)*x-2*1/k*b);
        gama = pow(((pow(k,2)+1)/k),2)*pow(x,2)+2*((pow(k,2)+1)/k)*b*x+pow(b,2)-2*x*y*k-2*x*y*1/k-2*y*b+pow(x,2)+ pow(y,2)-
            pow(d_offset,2);
        math::CommonMath commonMath;
        std::tuple<double,double> res = commonMath.univariateQuadraticEquation(alpha,beta,gama);
        x_ = std::max(std::get<1>(res),std::get<0>(res));
        const double xd = x_;
        const double yd = -1/k*xd+(k+1/k)*x+b;
        const double tangent = hdg + delta_s * curvature;
        PointENU pointEnu(xd,yd,0,s,point.hdg);
        s += delta_s;
#ifdef print_center
        LOG(ERROR) << xd << "," << yd<<","<<point.hdg;
#endif
        pointEnu.curveture = curvature;
        res_curveSegment.lineSegment.points.push_back(pointEnu);
      }
    }
    res_curveSegment.s = first_point.s;
    res_curveSegment.start_position = first_point;
    res_curveSegment.curveType = entity::CurveSegment::CurveType_ARC;
    res_curveSegment.heading = first_point.hdg;
  }
//    LOG(ERROR) << "--------------------- CENTER LINE END---------------------";
  laneInternal->lane.central_curve.segment.push_back(res_curveSegment);
  return 1;
}
}
}
}
