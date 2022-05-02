#include <fstream>

#include <random>
#include <cmath>
#include "modules/map/hdmap/hdmap_util.h"
#include "map_util/map_util.h"
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
#include "mutation_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "mutate_planning.h"
#include <iostream>
#include "modules/planning/proto/planning.pb.h"
#include <iomanip>
#define NUM_OF_OBJ 2

using apollo::localization::Pose;
using apollo::common::Point3D;
using apollo::perception::PerceptionObstacle;
using apollo::tools::fuzz::planning::OnLanePlanningFuzzMessage;
using apollo::prediction::PredictionObstacle;
using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::prediction::Trajectory;
using apollo::hdmap::GetHeading;
using apollo::hdmap::SLToXYZ;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
std::string ego_lane_id;
double ego_s, ego_l;


bool PlanningMutator::CheckBBoxOnRoad(PerceptionObstacle *input){
  std::vector<Point3D> corners = GetCorners(input);
  for (int i=0; i<4; i++){
    if (apollo::hdmap::PointIsOnRoad(corners[i])==true)
      return true;
  }
  return false;
}

void PlanningMutator::LoadBlackList(std::string config_path){
  std::ifstream ifs;
  ifs.open(config_path, std::ifstream::in); 
  int num;
  ifs>>num;
  std::string t_id;
  double d1,d2,d3,d4;
  for (int i=0; i<num; i++){
    ifs>>t_id>>d1>>d2>>d3>>d4;
    BlackArea temp_black;
    temp_black.lane_id = t_id;
    temp_black.min_s = d1;
    temp_black.max_s = d2;
    temp_black.min_l = d3;
    temp_black.max_l = d4;
    black_list.push_back(temp_black);
    
  }
  
  ifs.close();

}


void PlanningMutator::LoadBlackList(OnLanePlanningFuzzMessage *input, apollo::planning::ADCTrajectory *output){
  std::string curr_lane_id;
  apollo::common::PointENU XYPoint;
  XYPoint.set_x(input->localization_estimate().pose().position().x());
  XYPoint.set_y(input->localization_estimate().pose().position().y());
  apollo::hdmap::LaneInfoConstPtr nearest_lane; 
  double s, l;
  int temp = apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(XYPoint, &nearest_lane, &s, &l);
  curr_lane_id = nearest_lane->id().id();
  ego_s = s; ego_l = l;
  ego_lane_id = curr_lane_id;
  BlackArea temp_black;
  temp_black.lane_id = curr_lane_id; 
  temp_black.min_s = s - 2.0; 
  temp_black.max_s = nearest_lane->total_length(); 
//  temp_black.min_l = -nearest_lane->GetWidth(s)/2.0;
//  temp_black.max_l = nearest_lane->GetWidth(s)/2.0;
  temp_black.min_l = -1.30;
  temp_black.max_l = 1.30;
  AERROR<<curr_lane_id<<" "<<temp_black.min_l<<" "<<temp_black.max_l;
  black_list.push_back(temp_black); 
  
  AERROR<<"Load BlackList"; 
  for (int i=0; i<output->trajectory_point_size(); i++){
    XYPoint.set_x(output->trajectory_point(i).path_point().x());
    XYPoint.set_y(output->trajectory_point(i).path_point().y());
    temp = apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(XYPoint, &nearest_lane, &s, &l);
    std::string temp_lane_id = nearest_lane->id().id();
    AERROR<<temp_lane_id<<" "<<s<<" "<<l;
    bool lane_exist = false;
    for (int j=0; j<black_list.size(); j++)
      if (temp_lane_id == black_list[j].lane_id){
        if (s < black_list[j].min_s)
          black_list[j].min_s = s;
        if (s> black_list[j].max_s)
          black_list[j].max_s = s;
        lane_exist = true;
        break;
      }
    if (lane_exist == false){
       temp_black.lane_id = temp_lane_id; 
       temp_black.min_s = 0; 
       temp_black.max_s = nearest_lane->total_length(); 
       temp_black.min_l = -nearest_lane->GetWidth(s);
       temp_black.max_l = nearest_lane->GetWidth(s);
       black_list.push_back(temp_black);
    }
  }
}

bool PlanningMutator::CheckPoint(Point3D pos){
  for (int i=0; i<black_list.size(); i++ ){
    double s, l;
    apollo::hdmap::XYZToSL(black_list[i].lane_id, pos, &s, &l);
    if (s > black_list[i].max_s || s < black_list[i].min_s)
      continue;
    if (l > black_list[i].max_l || l < black_list[i].min_l)
      continue;
    return false;
  }
  return true;
}


bool PlanningMutator::CheckBBox(PerceptionObstacle *input){
  std::vector<Point3D> corners = GetCorners(input);
    for (int j=0; j<4; j++){
      if (CheckPoint(corners[j])==false)
        return false;
    }
  return true; 
}

//void MutatePos(Point3D *input2, Point3D *input1){
//}

bool PlanningMutator::VerifyPICons(OnLanePlanningFuzzMessage input){
  for (int i=0; i<NUM_OF_OBJ; i++){
    if (VerifyPICons(input.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)) == false)
      return false;
  }
  return true; 
}



bool PlanningMutator::VerifyPICons(PredictionObstacle *input){
  if (input->perception_obstacle().type()==apollo::perception::PerceptionObstacle::UNKNOWN_UNMOVABLE){
    bool feasible = CheckBBox(input->mutable_perception_obstacle());
    if (CheckBBoxOnRoad(input->mutable_perception_obstacle()) == true)
      feasible = false;
    return feasible;
  } 
  if (input->perception_obstacle().type() == apollo::perception::PerceptionObstacle::PEDESTRIAN){
    bool feasible = CheckBBox(input->mutable_perception_obstacle());
    if (CheckBBoxOnRoad(input->mutable_perception_obstacle()) == true)
      feasible = false;
    if (VerifyTrajectory(input)==false)
      feasible = false;
    return feasible;
  }
  if (input->perception_obstacle().type() == apollo::perception::PerceptionObstacle::VEHICLE){
    bool feasible = CheckBBox(input->mutable_perception_obstacle());
    if (CheckBBoxOnRoad(input->mutable_perception_obstacle()) == false)
      feasible = false;
    if (VerifyTrajectory(input)==false)
      feasible = false;
    return feasible;
  }


}


void PlanningMutator::GenerateInitialData(OnLanePlanningFuzzMessage *input, int enable_PI){
  double center_x = input->localization_estimate().pose().position().x();
  double center_y = input->localization_estimate().pose().position().y();
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  input->mutable_prediction_obstacles()->clear_prediction_obstacle();
  for (int i=0; i<NUM_OF_OBJ; i++){
    PredictionObstacle* temp = input->mutable_prediction_obstacles()->add_prediction_obstacle();
    temp->mutable_perception_obstacle()->set_id(i+1234);
    int random_int = std::rand()%3;
    if (random_int ==  0){
        if (enable_PI)
          do{
            InitVehicle(temp->mutable_perception_obstacle(), center_x, center_y, enable_PI);
            DrawTrajectory(temp);
          //AERROR<<"CP 1";
          } while (VerifyTrajectory(temp)==false);
        else
           InitVehicle(temp->mutable_perception_obstacle(), center_x, center_y, enable_PI); 
    }
    else if (random_int == 1){
      if (enable_PI)
      do{
        //AERROR<<"CP 0";
        //InitVehicle(temp->mutable_perception_obstacle(), center_x, center_y);
        InitPedestrian(temp->mutable_perception_obstacle(), center_x, center_y, enable_PI);
        DrawTrajectory(temp);
        //AERROR<<"CP 1";
        
      } while (VerifyTrajectory(temp)==false);
      else 
         InitPedestrian(temp->mutable_perception_obstacle(), center_x, center_y, enable_PI);
       
    } else{
      InitStatic(temp->mutable_perception_obstacle(),center_x, center_y, enable_PI);
      temp->set_is_static(true);
    }
    //DrawTrajectory(temp);
    //AERROR<<"Finish Gene Traj "<<i;
  }

}


void PlanningMutator::InitVehicle(PerceptionObstacle* x, double center_x, double center_y, int enable_PI){
  std::random_device rd;
  std::mt19937 gen(rd());
  AERROR<<center_x<<" "<<std::setprecision(9)<<center_y;
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  x->mutable_velocity()->set_x(0.0);
  x->mutable_velocity()->set_y(0.0);
  x->mutable_velocity()->set_z(0.0);
  x->set_length(4.70);
  x->set_width(2.06);
  x->set_height(2.06);
  x->set_type(apollo::perception::PerceptionObstacle::VEHICLE);
  
  bool feasible = false;
  while (feasible == false){
    double temp_x = center_x + unif(gen) * 160 - 80;
    double temp_y = center_y + unif(gen) * 160 - 80;
    Point3D XYPoint;
    XYPoint.set_x(temp_x);
    XYPoint.set_y(temp_y);
    XYPoint.set_z(0.0);
    double heading = apollo::hdmap::GetHeading(XYPoint);
    x->mutable_position()->set_x(temp_x);
    x->mutable_position()->set_y(temp_y);
    x->set_theta(heading);
    feasible = CheckBBox(x);
    //if (feasible == false )
    //  std::cout<<" The position is rejected by CheckBBox "<<std::setprecision(9)<<temp_x<<" "<<std::setprecision(9)<<temp_y<<"\n"
    if (CheckBBoxOnRoad(x) == false)
      feasible = false;
    if (!enable_PI)
      feasible=true;
  }
  double v_x = 6.8 * cos(x->theta());
  double v_y = 6.8 * sin(x->theta());
  x->mutable_velocity()->set_x(v_x);
  x->mutable_velocity()->set_y(v_y);
}


void PlanningMutator::InitStatic(PerceptionObstacle* x, double center_x, double center_y, int enable_PI){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  x->mutable_velocity()->set_x(0.0);
  x->mutable_velocity()->set_y(0.0);
  x->mutable_velocity()->set_z(0.0);
  x->set_length(0.8);
  x->set_width(0.8);
  x->set_height(0.8);
  
  //x->set_length(0.3 + unif(gen) * 0.4);
  //x->set_width(0.3 + unif(gen)* 0.4);
  //x->set_height(0.3 + unif(gen) * 0.4);
  x->set_type(apollo::perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  bool feasible = false;
  while (feasible == false){
    double temp_x = center_x + unif(gen) * 80 - 40;
    double temp_y = center_y + unif(gen) * 80 - 40;
    //if (587069 < temp_x && temp_x < 587070)
    //AERROR<<"Randomly init a static obs in middle\n";
    Point3D XYPoint;
    XYPoint.set_x(temp_x);
    XYPoint.set_y(temp_y);
    XYPoint.set_z(0.0);
    double heading = apollo::hdmap::GetHeading(XYPoint);
    x->mutable_position()->set_x(temp_x);
    x->mutable_position()->set_y(temp_y);
    x->set_theta(heading);
    TransportOffRd(x);
    feasible = CheckBBox(x);
    if (CheckBBoxOnRoad(x)==true)
      feasible = false;
    if (!enable_PI)
      feasible = true;
  std::cout<<"Forced obs position "<<feasible<<" "<<x->position().x() - center_x<<" "<<x->position().y() - center_y<<std::endl;
  } 
}


void PlanningMutator::InitPedestrian(PerceptionObstacle* x, double center_x, double center_y, int enable_PI){
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  x->mutable_velocity()->set_x(0.0);
  x->mutable_velocity()->set_y(0.0);
  x->mutable_velocity()->set_z(0.0);
  x->set_length(1.0);
  x->set_width(1.0);
  x->set_height(1.8);
  x->set_type(apollo::perception::PerceptionObstacle::PEDESTRIAN);
  bool feasible = false;
  while (feasible == false){
    double temp_x = center_x + unif(gen) * 160 - 80;
    double temp_y = center_y + unif(gen) * 160 - 80;
    Point3D XYPoint;
    XYPoint.set_x(temp_x);
    XYPoint.set_y(temp_y);
    XYPoint.set_z(0.0);
    double heading = apollo::hdmap::GetHeading(XYPoint);
    x->mutable_position()->set_x(temp_x);
    x->mutable_position()->set_y(temp_y);
    x->set_theta(heading);
    feasible = CheckBBox(x);
    if (CheckBBoxOnRoad(x)==true)
      feasible = false;
    if (!enable_PI)
      feasible = true;
  } 
  double v_x = 1.0 * cos(x->theta());
  double v_y = 1.0 * sin(x->theta());
  int rand_int = std::rand()%3;
  if (rand_int == 0){
    x->mutable_velocity()->set_x(v_x);
    x->mutable_velocity()->set_y(v_y);
  }
  else if (rand_int == 1){
    x->mutable_velocity()->set_x(-v_x);
    x->mutable_velocity()->set_y(-v_y);
    x->set_theta( x->theta()+ 3.14);
  }
  else{
    apollo::common::PointENU enu_point;
    enu_point.set_x(x->position().x());
    enu_point.set_y(x->position().y());
    apollo::hdmap::LaneInfoConstPtr nearest_lane;
    double s, l;
    int lane_id = apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(enu_point, &nearest_lane, &s, &l);
    if (l > 0 ){
      x->mutable_velocity()->set_x(- v_y);
      x->mutable_velocity()->set_y(v_x);
      x->set_theta(x->theta() + 1.57);
    }
    else{
      x->mutable_velocity()->set_x(v_y);
      x->mutable_velocity()->set_y(- v_x);
      x->set_theta(x->theta() - 1.57);
    }
  }
}


void PlanningMutator::MutatePlanningMsg(OnLanePlanningFuzzMessage *input, int enable_PI){
  int size = input->prediction_obstacles().prediction_obstacle_size();
  //if (size > 1){
  //}
  //else if (size < 4){
    
  //}
  //else{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  double center_x = input->localization_estimate().pose().position().x();
  double center_y = input->localization_estimate().pose().position().y();
  
  for (int i=0; i<size; i++){
    double rv = unif(gen); 
    if (rv < 0.9)
      MutatePredictionObstacle(input->mutable_prediction_obstacles()->mutable_prediction_obstacle(i), enable_PI);
    else if (rv >= 0.9 ){
      PredictionObstacle* temp = input->mutable_prediction_obstacles()->mutable_prediction_obstacle(i);
      temp->CopyFrom(PredictionObstacle());
      temp->mutable_perception_obstacle()->set_id(1234+i);
      int random_int = std::rand()%3;
      if (random_int ==  0){
        do{
          InitPedestrian(temp->mutable_perception_obstacle(), center_x, center_y, enable_PI);
          DrawTrajectory(temp);
        //AERROR<<"CP 1";
        } while (VerifyTrajectory(temp)==false);
      }
      else if (random_int == 1){
        do{
          InitPedestrian(temp->mutable_perception_obstacle(), center_x, center_y, enable_PI);
          AERROR<<"Init a vehicle with pos "<<temp->perception_obstacle().position().x()<<" "<<temp->perception_obstacle().position().y();
          DrawTrajectory(temp);
        
        } while (VerifyTrajectory(temp)==false);
      } else{
        InitStatic(temp->mutable_perception_obstacle(),center_x, center_y, enable_PI);
        temp->set_is_static(true);
      }
    }
  }
  //}  
}
void PlanningMutator::MutatePlanningMsgTrans(OnLanePlanningFuzzMessage *input){
  int size = input->prediction_obstacles().prediction_obstacle_size();
    for (int i=0; i<size; i++){
      MutatePredictionObstacleTrans(input->mutable_prediction_obstacles()->mutable_prediction_obstacle(i));
    }
}


void PlanningMutator::CrossoverPlanningMsg(OnLanePlanningFuzzMessage *input1, OnLanePlanningFuzzMessage *input2){
  for (int i=0; i<=1; i++){
    PredictionObstacle temp;
    temp.CopyFrom(input1->prediction_obstacles().prediction_obstacle(i));
    input1->mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->CopyFrom(input2->prediction_obstacles().prediction_obstacle(i));
    input2->mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->CopyFrom(temp);

  }
  return;
}

void PlanningMutator::MutatePosHeading(PerceptionObstacle* input){
  double previous_x = input->position().x();
  double previous_y = input->position().y();
  double previous_heading = input->theta();
  FMutate(&previous_x);
  FMutate(&previous_y);
  //double temp_x = previous_x + GetNormalDist();
  //double temp_y = previous_y + GetNormalDist();
  double temp_heading = apollo::hdmap::GetHeading(input->position());
  //input->mutable_position()->set_x(temp_x);
  //input->mutable_position()->set_y(temp_y);
  input->set_theta(temp_heading);
  input->mutable_position()->set_x(previous_x);
  input->mutable_position()->set_y(previous_y);
  //Transport(PerceptionObstacle* input);

}


void PlanningMutator::MutatePerceptionObstacle(PerceptionObstacle *input, int enable_PI){
  if (input->type() == apollo::perception::PerceptionObstacle::BICYCLE)  
    MutatePedestrian(input, enable_PI);  
  else if (input->type()==apollo::perception::PerceptionObstacle::VEHICLE)
    MutateVehicle(input, enable_PI);
  else if (input->type()== apollo::perception::PerceptionObstacle::UNKNOWN_UNMOVABLE)
    MutateStatic(input, enable_PI);
//MutatePosHeading(input);
  return;
}


Trajectory PlanningMutator::GenerateTrajectory(const std::vector<TrajectoryPoint>& points){
  Trajectory trajectory;
  trajectory.set_probability(1.0);
  *trajectory.mutable_trajectory_point() = {points.begin(), points.end()};
  return trajectory;  
}

void PlanningMutator::MutateVehicle(PerceptionObstacle* input, int enable_PI){
  bool feasible = false; AERROR<<"Start mutate vehicle";
  while (feasible == false){
    double temp_x = input->position().x();
    double temp_y = input->position().y();
    FMutate(&temp_x);
    FMutate(&temp_y);
    Point3D XYPoint;
    XYPoint.set_x(temp_x);
    XYPoint.set_y(temp_y);
    AERROR<<"The newly generated input is "<<temp_x<<" "<<temp_y;
    XYPoint.set_z(0.0);
    double heading = apollo::hdmap::GetHeading(XYPoint); 
    input->mutable_position()->set_x(temp_x);
    input->mutable_position()->set_y(temp_y);
    input->set_theta(heading);
    feasible = CheckBBox(input);
    if (feasible == false)
	AERROR<<"Check BBox  failed";
    if (CheckBBoxOnRoad(input) == false)
     feasible = false;
      if (!enable_PI)
      feasible = true;
  
  }
  double v_x = 6.8 * cos(input->theta());
  double v_y = 6.8 * sin(input->theta());
  input->mutable_velocity()->set_x(v_x);
  input->mutable_velocity()->set_y(v_y); AERROR<<"End mutate vehicle";
}
void PlanningMutator::MutateStatic(PerceptionObstacle *input, int enable_PI){
  bool feasible = false;  AERROR<<"Start mutate static";
  while (feasible == false){
    double temp_x = input->position().x();
    double temp_y = input->position().y();
    FMutate(&temp_x);
    FMutate(&temp_y);
    Point3D XYPoint;
    XYPoint.set_x(temp_x);
    XYPoint.set_y(temp_y);
    XYPoint.set_z(0.0);
    //input->set_theta(heading);
    //TransportOffRd(input);
    feasible = CheckBBox(input);
    if (CheckBBoxOnRoad(input)==true)
      feasible = false;
    if (!enable_PI)
      feasible = true;
  } AERROR<<"End mutate static";
}


void PlanningMutator::MutatePedestrian(PerceptionObstacle *input, int enable_PI){
  bool feasible = false; AERROR<<"Start muatate ped\n";
  while (feasible == false){
    double temp_x = input->position().x();
    double temp_y = input->position().y();
    FMutate(&temp_x);
    FMutate(&temp_y);
    Point3D XYPoint;
    XYPoint.set_x(temp_x);
    XYPoint.set_y(temp_y);
    XYPoint.set_z(0.0);
    double heading = apollo::hdmap::GetHeading(XYPoint);
    input->mutable_position()->set_x(temp_x);
    input->mutable_position()->set_y(temp_y);
    input->set_theta(heading);
    TransportOffRd(input);
    feasible = CheckBBox(input);
    if (CheckBBoxOnRoad(input)==true)
      feasible = false;
    if (!enable_PI)
      feasible = true;
  } 
  double v_x = 1.0 * cos(input->theta());
  double v_y = 1.0 * sin(input->theta());
  int rand_int = std::rand()%3;
  if (rand_int == 0){
    input->mutable_velocity()->set_x(v_x);
    input->mutable_velocity()->set_y(v_y);
  }
  else if (rand_int == 1){
    input->mutable_velocity()->set_x(-v_x);
    input->mutable_velocity()->set_y(-v_y);
    input->set_theta( input->theta()+ 3.14);
  }
  else{
    apollo::common::PointENU enu_point;
    enu_point.set_x(input->position().x());
    enu_point.set_y(input->position().y());
    apollo::hdmap::LaneInfoConstPtr nearest_lane;
    double s, l;
    int lane_id = apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(enu_point, &nearest_lane, &s, &l);
    if (l > 0 ){
      input->mutable_velocity()->set_x(- v_y);
      input->mutable_velocity()->set_y(v_x);
      input->set_theta(input->theta() + 1.57);
    }
    else{
      input->mutable_velocity()->set_x(v_y);
      input->mutable_velocity()->set_y(- v_x);
      input->set_theta(input->theta() - 1.57);
    }
  }
  AERROR<<"End mutate ped";
}

void PlanningMutator::MutatePerceptionObstacleTrans(PerceptionObstacle *input){
  Point3D prev;
  prev.CopyFrom(input->position());
  MutatePosHeading(input);
  auto transport_res = Transport(prev, input->position(), input->length()/2, input->width()/2);
  input->mutable_position()->set_x(transport_res[0].x());
  input->mutable_position()->set_y(transport_res[0].y());
  double temp_heading = apollo::hdmap::GetHeading(input->position());
  input->set_theta(temp_heading); 
  return;
}

std::vector<Point3D> PlanningMutator::GetCorners(const PerceptionObstacle *input){
  std::vector<Point3D> corners;
  double center_x = input->position().x();
  double center_y = input->position().y();
  double obs_l = input->length();
  double obs_w = input->width();
  double theta = input->theta();
  Point3D c1, c2, c3, c4;
  c1.set_x(center_x + (cos(theta) * obs_l - sin(theta)*obs_w)/2);
  c1.set_y(center_y + (sin(theta) * obs_l + cos(theta)*obs_w)/2);
  c2.set_x(center_x + (cos(theta) * (-obs_l) - sin(theta)*obs_w)/2);
  c2.set_y(center_y + ( sin(theta) * (-obs_l) + cos(theta)*obs_w)/2);
  c3.set_x(center_x + ( cos(theta) * (-obs_l) - sin(theta)*(-obs_w))/2);
  c3.set_y(center_y + (sin(theta) * (-obs_l) + cos(theta)*(-obs_w))/2);
  c4.set_x(center_x + (cos(theta) * obs_l - sin(theta)*(-obs_w))/2);
  c4.set_y(center_y + (sin(theta) * obs_l + cos(theta)*(-obs_w))/2);
  corners.push_back(c1);
  corners.push_back(c2);
  corners.push_back(c3);
  corners.push_back(c4);
  return corners;
}


//void PlanningMutator::CrossoverPredictionObstacle(PredictionObstacle *input1, PredictionObstacle *input2){
//  CrossoverPerceptionObstacle(input1->mutable_perception_obstacle(), input2->mutable_perception_obstacle());
//}

void PlanningMutator::MutatePredictionObstacle(PredictionObstacle *input, int enable_PI){
  MutatePerceptionObstacle(input->mutable_perception_obstacle(), enable_PI);
  DrawTrajectory(input);
  if (VerifyTrajectory(input) == false){
    MutatePredictionObstacle(input, enable_PI);
  }
}

void PlanningMutator::MutatePredictionObstacleTrans(PredictionObstacle *input){
  MutatePerceptionObstacleTrans(input->mutable_perception_obstacle());
}



double PlanningMutator::BarrierFunc(OnLanePlanningFuzzMessage *input){
  double ret = 0.0;
  for (int i=0; i<input->prediction_obstacles().prediction_obstacle_size(); i++){
    double temp_s, temp_l;
    GetMinDistFromBound(input->mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle(), &temp_s, &temp_l);
    ret += temp_s;
    ret += temp_l;
  }
  return ret;
}

double PlanningMutator::OnLaneBarrierFunc(OnLanePlanningFuzzMessage *input){
  double ret = 0.0;
  for (int i=0; i<input->prediction_obstacles().prediction_obstacle_size(); i++){
    double nearest_l = apollo::hdmap::GetNearestL(input->prediction_obstacles().prediction_obstacle(i).perception_obstacle().position(),
      input->prediction_obstacles().prediction_obstacle(i).perception_obstacle().width()/2.0);

    if (nearest_l >= 0.0)
      ret = std::numeric_limits<double>::infinity();
    else if (nearest_l > -0.2){
      ret = ret - std::log(nearest_l/(-0.2));
    }
  }
  std::cout<<"The OnLane barrier function is "<<ret<<'\n';
  return ret;
}


void PlanningMutator::GetMinDistFromBound(const PerceptionObstacle *input, double* barrier_s, double* barrier_l){
  std::cout<<"Start GetMinDist\n";
  auto corners = GetCorners(input);
  *barrier_s = 0.0; *barrier_l = 0.0;
  for (int i=0; i<black_list.size(); i++){
    double l_min, l_max, s_min, s_max;
    double temp_s, temp_l;
    apollo::hdmap::XYZToSL(black_list[i].lane_id, corners[0], &temp_s, &temp_l);
    l_min = temp_l; l_max = temp_l; s_min = temp_s; s_max = temp_s;
    for (int j=1; j<4; j++){
	    apollo::hdmap::XYZToSL(black_list[i].lane_id, corners[j], &temp_s, &temp_l);
      if (temp_s < s_min)
	s_min = temp_s;
      if (temp_s > s_max)
        s_max = temp_s;
      if (temp_l < l_min)
	l_min = temp_l;
      if (temp_l > l_max)
	l_max = temp_l;
    }
     
    double barrier_1 = GetBarrierValue(l_min, l_max, black_list[i].min_l, black_list[i].max_l);
    double barrier_2 = GetBarrierValue(s_min, s_max, black_list[i].min_s, black_list[i].max_s);
    
    std::cout<<"barrier_1 "<<barrier_1<<" "<<l_min<<" "<<l_max<<" "<<black_list[i].min_l<<" "<<black_list[i].max_l<<'\n';
    std::cout<<"barrier_2 "<<barrier_2<<" "<<s_min<<" "<<s_max<<" "<<black_list[i].min_s<<" "<<black_list[i].max_s<<'\n';
    
    if (barrier_1 < barrier_2)
      (*barrier_s) = (*barrier_s) + barrier_1;
    else
      (*barrier_l) = (*barrier_l) + barrier_2;
    std::cout<<"after update: "<<*barrier_s<<" "<<*barrier_l<<'\n';
  }
  std::cout<<"EndMinDist\n";
  return;
}

double PlanningMutator::GetBarrierValue(double x, double y, double con1, double con2){
  if (y < con1 - 1.0) 
    return 0.0;
  else if (x > con2 + 1.0)
    return 0.0;
  else if ( y >= con1-1.0 && y<con1)
    return -std::log(con1 - y);
  else if (x> con2 && x<=con2 + 1.0)
    return -std::log(x - con2);
  else
    return std::numeric_limits<double>::infinity();
}

int PlanningMutator::InferDirection(const Point3D prev, const Point3D next, int obs_idx, double buffer_s, double buffer_l){
  int ret = 0;
  double prev_s, prev_l, next_s, next_l;
  apollo::hdmap::XYZToSL(black_list[obs_idx].lane_id, prev, &prev_s, &prev_l);
  apollo::hdmap::XYZToSL(black_list[obs_idx].lane_id, next, &next_s, &next_l);
  if (prev_s < black_list[obs_idx].min_s - buffer_s && next_s >= black_list[obs_idx].min_s - buffer_s)
    ret +=0x1;
  if (prev_s > black_list[obs_idx].max_s + buffer_s && next_s <= black_list[obs_idx].max_s + buffer_s)
    ret +=0x3;
  if (prev_l < black_list[obs_idx].min_l - buffer_l && next_l >= black_list[obs_idx].min_l - buffer_l)
    ret +=0x7;
  if (prev_l > black_list[obs_idx].max_l + buffer_l && next_l <= black_list[obs_idx].max_l + buffer_l)
    ret += 0xf;
  return ret;
}


void PlanningMutator::TransportOffRd(PerceptionObstacle *input){
  std::vector<Point3D> corners  = GetCorners(input);
  std::string lane_id; double s, l;
  LaneInfoConstPtr lane_info;
  bool is_on_lane = false;
  for (int i=0; i<4; i++){
    apollo::common::PointENU temp_enu; 
    temp_enu.set_x(corners[i].x());
    temp_enu.set_y(corners[i].y());
    apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(temp_enu, &lane_info, &s, &l);
    lane_id  = lane_info->id().id();
    AERROR<<lane_id<<" "<<s<<" "<<l;
    if (lane_info->IsOnLane(apollo::common::math::Vec2d(input->position().x(),
		input->position().y()))== true){
       AERROR<<"This point is onlane and execute transport";
       is_on_lane = true;
       break;
    }
  }
  if (is_on_lane  == false)
    return;
  double l_min, l_max, s_min, s_max;
  for (int i=0; i<4; i++){
    double temp_s, temp_l;
    lane_info->GetProjection((apollo::common::math::Vec2d(corners[i].x(),
		corners[i].y())), &temp_s, &temp_l);
    if (i==0){
      l_min = temp_l; l_max = temp_l;
      s_min = temp_s; s_max = temp_s;
    }
    else {
      if (temp_s < s_min)  s_min = temp_s;
      if (temp_s > s_max)  s_max = temp_s;
      if (temp_l < l_min)  l_min = temp_l;
      if (temp_l > l_max)  l_max = temp_l;
    }
  }
  AERROR<<l_min<<" "<<l_max<<" "<<s_min<<" "<<s_max;
  std::vector<Point3D> trans_points;
  double left_width, right_width;
  lane_info->GetWidth((s_min+s_max)/2.0, &left_width, &right_width);
  left_width = - left_width;
  double l_center;
  //if (l_min + l_max < 0){
  l_center = l_min - left_width + right_width + (l_max - l_min)/2.0; 
  //}
  //else{
   // l_center = (l_min + l_max)/2.0 + (left_width - right_width);
  //}
  double s_center = (s_min + s_max)/2.0; 
  auto temp_point = SLToXYZ(lane_id, s_center, l_center);
  Point3D new_point;
  new_point.set_x(temp_point.x());
  new_point.set_y(temp_point.y());
  trans_points.push_back(new_point);

  l_center = (l_min + l_max)/2.0 + (left_width - right_width);
  s_center = (s_min + s_max)/2.0; 
  temp_point = SLToXYZ(lane_id, s_center, l_center);
  new_point.set_x(temp_point.x());
  new_point.set_y(temp_point.y());
  trans_points.push_back(new_point);


  double s_length = lane_info->total_length();
  if (s_max < 2){
    l_center = (l_min + l_max)/2.0;
    s_center = (s_min + s_max)/2.0 + s_length;
    auto temp_point = SLToXYZ(lane_id, s_center, l_center);
    new_point.set_x(temp_point.x());
    new_point.set_y(temp_point.y());
    trans_points.push_back(new_point);
  }
  if (s_min > s_length - 2){
    l_center = (l_min + l_max)/2.0;
    s_center = (s_min + s_max)/2.0 - s_length;
    auto temp_point = SLToXYZ(lane_id, s_center, l_center);
    new_point.set_x(temp_point.x());
    new_point.set_y(temp_point.y());
    trans_points.push_back(new_point);
  }
  int rand_num = std::rand()%trans_points.size();
  input->mutable_position()->set_y(trans_points[rand_num].y());
  input->mutable_position()->set_x(trans_points[rand_num].x());
  input->set_theta(GetHeading(input->position()));
  TransportOffRd(input);
}

int PlanningMutator::CheckPointWithBuffer(const Point3D pos, double buffer_s, double buffer_l){
   if (pos.x() > 587070 && pos.x() < 587073)
     AERROR<<"A point in right lane CP1";
   for (int i=0; i<black_list.size(); i++ ){
    double s, l;
    apollo::hdmap::XYZToSL(black_list[i].lane_id, pos, &s, &l);
     if (pos.x() > 587070 && pos.x() < 587073){
      AERROR<<"Point in the right lane is "<<pos.x()<<" "<<pos.y();
      AERROR<<black_list[i].lane_id<<" "<<s<<" "<<l;
    }
    if (s > black_list[i].max_s + buffer_s || s < black_list[i].min_s - buffer_s)
      continue;
    if (l > black_list[i].max_l + buffer_l || l < black_list[i].min_l - buffer_l)
      continue;
    return i;
  }
  return -1;
}



std::vector<Point3D> PlanningMutator::Transport(const Point3D prev, const Point3D next, double buffer_s, double buffer_l){
  std::vector<Point3D> ret;
  ret.clear();
  AERROR<<"Get into Transport func";
  int obs_idx = CheckPointWithBuffer(next, buffer_s, buffer_l);
  if (obs_idx == -1){
    ret.push_back(next);
    return ret;
  }
  int direction = InferDirection(prev, next, obs_idx, buffer_s, buffer_l);
  for (int direction_idx=0; direction_idx<4; direction_idx++){
    if (direction & (0x1 << direction_idx)){
      int temp_direction = direction & (0x1 << direction_idx);
      Point3D temp_prev, temp_next; 
      temp_prev.CopyFrom(prev);
      temp_next.CopyFrom(next);
      int temp_idx = obs_idx;
      while (temp_idx!=-1){
        temp_prev.CopyFrom(temp_next);
        Transport(&temp_next, temp_idx, temp_direction, buffer_s, buffer_l);
        temp_idx = CheckPointWithBuffer(temp_next, buffer_s, buffer_l);
        if (temp_idx!=-1)
          temp_direction = InferDirection(temp_prev, temp_next, temp_idx, buffer_s, buffer_l);
      }
      ret.push_back(temp_next);
    }
  }
  AERROR<<"Successfully transport point "<<prev.x()<<" "<<std::setprecision(7)<<prev.y()<<" "<<ret[0].x()<<" "<<std::setprecision(7)<<ret[0].y();
  return ret;
}



/*
std::vector<OnLanePlanningFuzzMessage> PlanningMutator::Transport(OnLanePlanningFuzzMessage input){
  bool feasible = true;
  for (int i=0; i<input.prediction_obstacles.prediction_obstacle_size(); i++)
  if (!CheckBBox(input.mutable_perception_obstacles().mutable_prediction_obstacle(i).perception_obstacle)){
    feasible = false;
  }
  std::vector<OnLanePlanningFuzzMessage> ret;
  ret.clear();
  if (feasible){
    ret.push_back(input);
    return ret;
  }
  else{
    	
  }
}
*/

void PlanningMutator::Transport(Point3D *pos, int obs_idx, int direction, double buffer_s, double buffer_l){
  if (direction == 0x1){
    double s, l; 
    apollo::hdmap::XYZToSL(black_list[obs_idx].lane_id, *pos, &s, &l);
    s = black_list[obs_idx].max_s + (s - (black_list[obs_idx].min_s - buffer_s)); 
    const auto XYPoint = apollo::hdmap::SLToXYZ(black_list[obs_idx].lane_id, s, l);
    pos->set_x(XYPoint.x());
    pos->set_y(XYPoint.y());
  }	  
   if (direction == 0x3){
    double s, l; 
    apollo::hdmap::XYZToSL(black_list[obs_idx].lane_id, *pos, &s, &l);
    s = black_list[obs_idx].min_s - ((black_list[obs_idx].max_s + buffer_s) - s); 
    const auto XYPoint = apollo::hdmap::SLToXYZ(black_list[obs_idx].lane_id, s, l);
    pos->set_x(XYPoint.x());
    pos->set_y(XYPoint.y());
  }
  if (direction == 0x7){
    double s, l; 
    apollo::hdmap::XYZToSL(black_list[obs_idx].lane_id, *pos, &s, &l);
    l = black_list[obs_idx].max_l + (l - (black_list[obs_idx].min_l - buffer_l)); 
    const auto XYPoint = apollo::hdmap::SLToXYZ(black_list[obs_idx].lane_id, s, l);
    pos->set_x(XYPoint.x());
    pos->set_y(XYPoint.y());
  }if (direction == 0xf){
    double s, l; 
    apollo::hdmap::XYZToSL(black_list[obs_idx].lane_id, *pos, &s, &l);
    l = black_list[obs_idx].min_l - ((black_list[obs_idx].max_l + buffer_l) - l); 
    const auto XYPoint = apollo::hdmap::SLToXYZ(black_list[obs_idx].lane_id, s, l);
    pos->set_x(XYPoint.x());
    pos->set_y(XYPoint.y());
  }
}


/*
void PlanningMutator::Transport(PerceptionObstacle* input){
  std::vector corners = GetCorners(input);
  for (int i=0; i<black_list.size(); i++){
    double bb_min_s = std::numeric_limits<double>::infinity();
    double bb_min_l = std::numeric_limits<double>::infinity();
    double bb_max_s = -std::numeric_limits<double>::infinity();
    double bb_max_l = -std::numeric_limits<double>::infinity();
    
    for (int j=0; j< 4; j++){
      double s, l;
      XYZToSL(black_list[i].lane_id, corners[j], &s, &l);
      if (s < bb_min_s) bb_min_s = s;
      if (s > bb_max_s) bb_max_s = s;
      if (l < bb_min_l) bb_min_l = l;
      if (l > bb_max_l) bb_max_l = l;
    }
    if (() &&
	())
    
  }
  return;
}
*/


bool PlanningMutator::VerifyTrajectory(PredictionObstacle* input){
  if (input->trajectory_size()==0)
    return true;
  if (input->perception_obstacle().type() == apollo::perception::PerceptionObstacle::VEHICLE){
    apollo::common::PointENU temp_enu;
    temp_enu.set_x(input->trajectory(0).trajectory_point(0).path_point().x());
    temp_enu.set_y(input->trajectory(0).trajectory_point(0).path_point().y());
    apollo::hdmap::LaneInfoConstPtr lane_info;
    double s,l;
    apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(temp_enu, &lane_info, &s, &l);
    AERROR<<"Verify the traj of a vehicle "<<lane_info->id().id()<<" "<<s<<" "<<l;
    AERROR<<"Ego position is "<<ego_lane_id<<" "<<ego_s<<" "<<ego_l;
    if (lane_info->id().id() == ego_lane_id && s < ego_s- 2.0)
      return true;
    if (lane_info->id().id() == ego_lane_id && s < ego_s + 20 && s > ego_s +10){
      input->clear_trajectory();
      return true;
    }
  }
  for (int i=0; i<input->trajectory(0).trajectory_point_size(); i++){
    Point3D pos;
    pos.set_x(input->trajectory(0).trajectory_point(i).path_point().x());
    pos.set_y(input->trajectory(0).trajectory_point(i).path_point().y());
    if (CheckPoint(pos) == false ){
      return false;
     }
  }
  return true;
}
void PlanningMutator::DrawTrajectory(PredictionObstacle* input){
  if (input->perception_obstacle().type() == apollo::perception::PerceptionObstacle::BICYCLE){
    std::vector<TrajectoryPoint> points;
    for (int i=0; i<160; i++){
      double relative_time = i* 0.1;
      TrajectoryPoint trajectory_point; 
      PathPoint path_point;
      path_point.set_x(input->perception_obstacle().position().x() + input->perception_obstacle().velocity().x() * relative_time);
      path_point.set_y(input->perception_obstacle().position().y() + input->perception_obstacle().velocity().y() * relative_time);
      path_point.set_z(0.0);
      path_point.set_theta(input->perception_obstacle().theta());
      trajectory_point.mutable_path_point()->CopyFrom(path_point);
      trajectory_point.set_v(1.0);
      trajectory_point.set_a(0.0);
      trajectory_point.set_relative_time(relative_time);
      points.emplace_back(std::move(trajectory_point));
    }
    Trajectory trajectory = GenerateTrajectory(points);
    input->clear_trajectory();
    input->add_trajectory()->CopyFrom(trajectory);
  }
  else if (input->perception_obstacle().type() == apollo::perception::PerceptionObstacle::VEHICLE){
     std::vector<TrajectoryPoint> points;
     std::random_device rd;
     std::mt19937 gen(rd());
     std::uniform_real_distribution<double> unif(0.0, 1.2);
     double curr_s, curr_l; std::string lane_id;
     apollo::hdmap::LaneInfoConstPtr nearest_lane;
     apollo::common::PointENU enu_point;
     enu_point.set_x(input->perception_obstacle().position().x());
     enu_point.set_y(input->perception_obstacle().position().y());
     apollo::hdmap::HDMapUtil::BaseMap().GetNearestLane(enu_point, &nearest_lane, &curr_s, &curr_l);
     lane_id = nearest_lane->id().id();
     AERROR<<"The pos of vehicle is "<<input->perception_obstacle().position().x()<<" "<<std::setprecision(7)<<input->perception_obstacle().position().y();
     AERROR<<"The point of vehicle before drawing the traj "<<lane_id<<" "<<curr_s<<" "<<curr_l;
     if (curr_l< -0.6|| curr_l >0.6 )
       curr_l = unif(gen) - 0.6;
     //apollo::hdmap::XYZToSL(lane_id, input->perception_obstacle().position(), &curr_s, &curr_l);
     for (int i=0; i < 100; i++){
       double relative_time = i * 0.1; 
       TrajectoryPoint trajectory_point; 
       PathPoint path_point;
       path_point.set_x(apollo::hdmap::SLToXYZ(lane_id, curr_s, curr_l).x());
       path_point.set_y(apollo::hdmap::SLToXYZ(lane_id, curr_s, curr_l).y());
       if (i == 0){
	 AERROR<<path_point.x()<<" "<<path_point.y();
         input->mutable_perception_obstacle()->mutable_position()->set_x(path_point.x());
         input->mutable_perception_obstacle()->mutable_position()->set_y(path_point.y());
       }
       path_point.set_z(0.0);
       path_point.set_theta(apollo::hdmap::GetHeading(lane_id, curr_s));
       trajectory_point.mutable_path_point()->CopyFrom(path_point);
       trajectory_point.set_v(6.8);
       trajectory_point.set_a(0.0);
       trajectory_point.set_relative_time(relative_time);
       points.emplace_back(std::move(trajectory_point));

       curr_s = curr_s + 6.8 * 0.1;
       if (curr_s > apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(apollo::hdmap::MakeMapId(lane_id))->total_length()){
         curr_s = curr_s - apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(apollo::hdmap::MakeMapId(lane_id))->total_length();
         auto temp_lane_ptr = apollo::hdmap::PncMap(&apollo::hdmap::HDMapUtil::BaseMap()).GetRouteSuccessor(
           apollo::hdmap::HDMapUtil::BaseMap().GetLaneById(apollo::hdmap::MakeMapId(lane_id)));
         if (temp_lane_ptr == NULL)
           break;
         else 
           lane_id = temp_lane_ptr->id().id();
       }
     }
     Trajectory trajectory = GenerateTrajectory(points);
     input->clear_trajectory();
     input->add_trajectory()->CopyFrom(trajectory);
  }
  
}
