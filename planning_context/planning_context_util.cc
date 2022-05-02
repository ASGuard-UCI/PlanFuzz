#include "planning_context_util.h"
#include <fstream>
#include <string>
#include "modules/tools/fuzz/vuln_test/planning_context/planning_context.pb.h"
#include "modules/planning/common/planning_context.h"

using apollo::tools::fuzz::planning::PlanningContextPb;

bool CompareContext(const PlanningContextPb a, const PlanningContextPb b){
  if (a.planning_status().has_stop_sign() && b.planning_status().has_stop_sign()){
    int num_1 = a.planning_status().stop_sign().wait_for_obstacle_id_size();
    int num_2 = b.planning_status().stop_sign().wait_for_obstacle_id_size();
    if (num_1!=num_2)
      return false;   
  }
  return true;

}

void StorePlanningContext(PlanningContextPb *x){
  x->mutable_planning_status()->CopyFrom(apollo::planning::PlanningContext::Instance()->planning_status());
  x->mutable_path_decider_info()->CopyFrom(apollo::planning::PlanningContext::Instance()->path_decider_info());
  x->mutable_fall_back_info()->set_path_label(apollo::planning::PlanningContext::Instance()->mutable_fallback_info()->last_successful_path_label);
  
  //x->mutable_open_space_info()->Clear();
  //OpenSpaceInfo *temp = apollo::planning::PlanningContext::Instance()->mutable_open_space_info();
  //for (int i=0; i<temp->size(); i++){

  //}
}

void LoadPlanningContext(PlanningContextPb x){
  apollo::planning::PlanningContext::Instance()->mutable_planning_status()->CopyFrom(x.planning_status());
  apollo::planning::PlanningContext::Instance()->mutable_path_decider_info()->CopyFrom(x.path_decider_info());
  

}

void StorePlanningContext(std::string path){
  PlanningContextPb temp_pb;
  StorePlanningContext(&temp_pb);
  std::ofstream output(path, std::ios::out| std::ios::trunc|std::ios::binary);
  temp_pb.SerializeToOstream(&output);
  return;
}

void LoadPlanningContext(std::string path){
  std::ifstream input(path, std::ios::in|std::ios::binary);
  PlanningContextPb temp_pb;
  temp_pb.ParseFromIstream(&input);
  LoadPlanningContext(temp_pb);
  return;
}

