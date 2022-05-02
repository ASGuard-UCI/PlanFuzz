#include "modules/map/hdmap/hdmap_util.h"
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
namespace apollo{
namespace hdmap{

  apollo::common::PointENU SLToXYZ(const std::string& lane_id, const double s, const double l){
    const auto lane_info = HDMapUtil::BaseMap().GetLaneById(MakeMapId(lane_id));
//    AERROR<<lane_id<<" "<<s<<" "<<l;
    return lane_info->GetSmoothPoint(s, l);
  }
  double GetHeading(const std::string& lane_id, const double s){
    const auto lane_info = HDMapUtil::BaseMap().GetLaneById(MakeMapId(lane_id));
    return lane_info->Heading(s);
  }
 
  double GetHeading(const apollo::common::Point3D pos){
    apollo::common::PointENU XYPoint;
    XYPoint.set_x(pos.x());
    XYPoint.set_y(pos.y());
    LaneInfoConstPtr nearest_lane;
    double s, l;
    int lane_id = HDMapUtil::BaseMap().GetNearestLane(XYPoint, &nearest_lane, &s, &l);
    return nearest_lane->Heading(s);

  }

  double GetNearestL(const apollo::common::Point3D pos, double buffer_l){
    apollo::common::PointENU XYPoint;
    XYPoint.set_x(pos.x());
    XYPoint.set_y(pos.y());
    LaneInfoConstPtr nearest_lane;
    double s, l;
    int lane_id = HDMapUtil::BaseMap().GetNearestLane(XYPoint, &nearest_lane, &s, &l);
    double left_width, right_width;
    nearest_lane->GetWidth(s, &left_width, &right_width);
    if ( -right_width + buffer_l < l && l < left_width - buffer_l){
      double temp1 = l - (-right_width + buffer_l);
      double temp2 = (left_width - buffer_l) - l;
      if (temp1< temp2)
        return -temp1;
      else
	return -temp2;

    }
    else if (l < -right_width + buffer_l)
      return ((-right_width + buffer_l) - l);
    else
      return (l - (left_width - buffer_l));
  }

  void XYZToSL(const std::string& lane_id, const apollo::common::Point3D pos, double *s, double *l){
    const auto lane_info = HDMapUtil::BaseMap().GetLaneById(MakeMapId(lane_id));
    apollo::common::math::Vec2d pos_vec2d(pos.x(), pos.y());
    lane_info->GetProjection(pos_vec2d, s, l);
    if (*s == 0){
      apollo::common::PointENU base_point;
      base_point = SLToXYZ(lane_id, 0, *l); 
      *s = - std::sqrt((base_point.x() - pos.x())*(base_point.x() - pos.x()) + (base_point.y() - pos.y())*(base_point.y() - pos.y()));
    }
     else if (*s == lane_info->total_length()){
      apollo::common::PointENU base_point;
      base_point = SLToXYZ(lane_id, lane_info->total_length(), *l); 
      *s = *s + std::sqrt((base_point.x() - pos.x()) * (base_point.x() - pos.x()) + (base_point.y() - pos.y()) * (base_point.y() - pos.y()));
    }
    

    return;
  }

  bool PointIsOnRoad(const apollo::common::Point3D pos){
    apollo::common::SLPoint sl_point;
    apollo::common::PointENU enu_point;
    enu_point.set_x(pos.x());
    enu_point.set_y(pos.y());
    apollo::common::math::Vec2d pos_vec2d(pos.x(), pos.y());
    std::vector<RoadInfoConstPtr> near_roads;
    int success = HDMapUtil::BaseMap().GetRoads(enu_point, 10, &near_roads);
    if (success == -1 )
      return false;
    else{
      for (int i=0; i< near_roads.size(); i++){
        for (int j=0; j< near_roads[i]->sections().size(); j++){
          RoadSection temp_section  = near_roads[i]->sections()[j];
          for (int k=0; k< temp_section.lane_id_size(); k++){
            const auto lane_info = HDMapUtil::BaseMap().GetLaneById(temp_section.lane_id(k));
            if (lane_info->IsOnLane(pos_vec2d)==true)
              return true;
          }
        }
      }
    }
    return false;
  }


} //end of namespace hdmap
} //end of namespace apollo
