#include <string>
#include <iostream>
#include <fstream>
#include "modules/tools/fuzz/vuln_test/opt/openGA/openGA.hpp"

#include "modules/tools/fuzz/planning/on_lane_planning_fuzz.h"
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/tools/fuzz/vuln_test/tools/fitness_tracker.h"
#include "modules/planning/common/planning_context.h"
#include "modules/tools/fuzz/vuln_test/mutation_v1/mutate_planning.h"
FitnessTracker fitness_tracker;
int case_index = 0;
PlanningMutator mutator;
apollo::planning::PathDeciderInfo stored_path_decider_info;
apollo::planning::PlanningStatus stored_planning_status;
std::string input_debug_path = "/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/data/inputdebug";
std::string input_msg_path = "/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/data/inputmsg";
std::string output_debug_path = "/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/data/outputdebug";
std::string output_traj_path = "/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/data/outputtraj";
using apollo::planning::ADCTrajectory;
using apollo::tools::fuzz::planning::OnLanePlanningFuzzMessage;
using apollo::planning::fuzzing::OnLanePlanningFuzzer;
std::ofstream *log_pointer;
OnLanePlanningFuzzer fuzzer;
struct MySolution{
  OnLanePlanningFuzzMessage input;
  std::string to_string() const{
    return input.DebugString();
  }
  std::vector<double> s_vec;
  std::vector<double> l_vec;

};

struct MyMiddleCost{
  
  double fitness;
};

typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type; 
GA_Type ga_obj;
typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_Type;

void copy_context(){
  int fd = open("/apollo/seeds/planning/lane_borrow", O_RDONLY);
  google::protobuf::io::FileInputStream raw_input(fd);
  OnLanePlanningFuzzMessage temp_input;
  google::protobuf::TextFormat::Parse(&raw_input, &(temp_input));
  //mutator.GenerateInitialData(&(p.input));
  //p.input.ParseFromIstream(&input_path);
  //std::cout<<"begin print pb message"<<std::endl;
  //std::cout<<p.input.DebugString()<<std::endl;
  ADCTrajectory temp;
  fuzzer.GetResult(&temp_input, &temp);
  AERROR<<"Counter : "<<apollo::planning::PlanningContext::Instance()->path_decider_info().front_static_obstacle_cycle_counter();
  fuzzer.GetResult(&temp_input, &temp);
  AERROR<<"Counter : "<<apollo::planning::PlanningContext::Instance()->path_decider_info().front_static_obstacle_cycle_counter();
  
  fuzzer.GetResult(&temp_input, &temp);
  AERROR<<"Counter : "<<apollo::planning::PlanningContext::Instance()->path_decider_info().front_static_obstacle_cycle_counter();
  //fuzzer.GetResult(&temp_input, &temp);
  //AERROR<<"Counter : "<<apollo::planning::PlanningContext::Instance()->path_decider_info().front_static_obstacle_cycle_counter();
  stored_planning_status.CopyFrom(apollo::planning::PlanningContext::Instance()->planning_status());
  stored_path_decider_info.CopyFrom(apollo::planning::PlanningContext::Instance()->path_decider_info());
  AERROR<<stored_path_decider_info.DebugString();
  close(fd);
  return;
}


void init_genes(MySolution& p, const std::function<double(void)> &rnd01){
  int fd = open("/apollo/seeds/planning/lane_borrow", O_RDONLY);
  google::protobuf::io::FileInputStream raw_input(fd);
  google::protobuf::TextFormat::Parse(&raw_input, &(p.input));
  //mutator.GenerateInitialData(&(p.input));
  //p.input.ParseFromIstream(&input_path);
  //std::cout<<"begin print pb message"<<std::endl;
  //std::cout<<p.input.DebugString()<<std::endl;
  
  
  mutator.GenerateInitialData(&(p.input));
  close(fd);
  //std::ofstream output("/apollo/modules/tools/fuzz/vuln_test/scenario/crash/data/planningmsg0", std::ios::out|std::ios::trunc|std::ios::binary);
  //p.input.SerializeToOstream(&output);
  //output.close();
}

bool eval_solution(const MySolution& p, MyMiddleCost &c){
  ADCTrajectory* temp = new ADCTrajectory;
  AERROR<<stored_path_decider_info.DebugString();
  refresh_shared_mem();
  //fitness_tracker.Init(log_pointer);
  OnLanePlanningFuzzMessage tmp_msg;
    tmp_msg.CopyFrom(p.input);
  AERROR<<stored_path_decider_info.DebugString();
  /*for (int i=0; i< tmp_msg.prediction_obstacles().prediction_obstacle_size(); i++){
    tmp_msg.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle()->set_theta(0.0);
     tmp_msg.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle()->mutable_velocity()->set_y(0.0);
     tmp_msg.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle()->mutable_velocity()->set_x(0.0);
    
  }*/
  apollo::planning::PlanningContext::Instance()->mutable_planning_status()->CopyFrom(stored_planning_status);
  apollo::planning::PlanningContext::Instance()->mutable_path_decider_info()->CopyFrom(stored_path_decider_info);
  std::cout<<stored_path_decider_info.DebugString();
  AERROR<<"Start executing case "<<case_index<<'\n';
  fuzzer.GetResult(&tmp_msg, temp);
  std::cout<<"Getting fitness value for "<<case_index<<'\n';
  c.fitness = get_fitness_shared_mem();
  //std::cout<<temp->DebugString()<<"\n";
  //c.fitness =fitness_tracker.GetFitness();
  //
  std::stringstream ss;
  ss<<output_traj_path<<case_index;
  std::ofstream output(ss.str(), std::ios::out|std::ios::trunc|std::ios::binary);
  temp->SerializeToOstream(&output);
  output.close();
  ss.str(std::string());
  ss<<input_msg_path<<case_index;
  std::ofstream msg_out(ss.str(), std::ios::out|std::ios::trunc|std::ios::binary);
  tmp_msg.SerializeToOstream(&msg_out);
  msg_out.close();
  for (int i=0; i< temp->decision().object_decision().decision_size(); i++)
    if (temp->decision().object_decision().decision(i).perception_id() == 3 &&
         temp->decision().object_decision().decision(i).object_decision(0).has_nudge()==false){

    std::cout<<"Find an input without nudge decision\n";
    ga_obj.user_request_stop = true;
  }
  /*if (temp->decision().main_decision().has_stop())
    if (temp->decision().main_decision().stop().change_lane_type()!= 1){ 
      std::cout<<"Program stop because of finding a vuln!\n";
      ga_obj.user_request_stop = true;
  }
  if (temp->decision().main_decision().has_cruise())
    if (temp->decision().main_decision().cruise().change_lane_type()!= 1){ 
      std::cout<<"Program stop because of finding a vuln!\n";
      ga_obj.user_request_stop = true;
  }*/

  case_index++;
  return true;
}

MySolution crossover(const MySolution& X1, const MySolution&X2, const std::function<double(void)> &rnd01){
  MySolution temp_s1, temp_s2;
  temp_s1.input.CopyFrom(X1.input);
  temp_s2.input.CopyFrom(X2.input);
  mutator.CrossoverPlanningMsg(&temp_s1.input, &temp_s2.input);
  return X1; 
}

MySolution mutate(const MySolution& X_base, const std::function<double(void)> &rnd01, double shrink_scale){
  MySolution X_ret;
  X_ret.input.CopyFrom(X_base.input);
  mutator.MutatePlanningMsg(&X_ret.input);
  return X_ret;
}

double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X){
  return X.middle_costs.fitness;
}
std::ofstream output_file;
void SO_report_generation(
	int generation_number, 
	const EA::GenerationType<MySolution, MyMiddleCost> &last_generation, 
	const MySolution& best_genes){
  std::cout
    <<"Generation ["<<generation_number<<"], "
    <<"Best = "<<last_generation.best_total_cost<<", "
    <<"Average = "<<last_generation.average_cost<<", "
    <<"Exe_time = "<<last_generation.exe_time
    <<std::endl;

  output_file
    <<generation_number<<"\t"
    <<last_generation.average_cost<<"\t"
    <<last_generation.best_total_cost<<"\n";
}

int main(int argc, char** argv){
  FLAGS_map_dir = "/apollo/modules/map/data/single_lane_road";
  output_file.open("./result_so1.txt");
  std::ofstream log_file("log_fit.txt", std::ios::out);
  log_pointer = &log_file;
  
  std::cout<<"Beginning of OpenGA process"<<std::endl;
  fuzzer.Init(argc, argv);
  //fuzzer.Warmup();
  std::cout<<"Finisht the warmup of fuzzer"<<std::endl;
 
  copy_context();
  mutator.LoadBlackList("/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/lane_borrow_blacklist");
  fitness_tracker.Init(&log_file);
  init_shared_mem();
  EA::Chronometer timer;
  timer.tic();
  LoadInstInfo("/apollo/vuln_test_bc/lane_borrow/inst_config");
  //GA_Type ga_obj;
  ga_obj.problem_mode = EA::GA_MODE::SOGA;
  ga_obj.multi_threading=false;
  ga_obj.idle_delay_us = 1;
  ga_obj.verbose =false;
  ga_obj.population = 20;
  ga_obj.generation_max = 100;
  ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
  ga_obj.init_genes = init_genes;
  ga_obj.eval_solution = eval_solution;
  ga_obj.mutate = mutate;
  ga_obj.crossover = crossover;
  ga_obj.SO_report_generation = SO_report_generation;
  ga_obj.best_stall_max = 50;
  ga_obj.average_stall_max = 50;
  ga_obj.elite_count = 10;
  ga_obj.crossover_fraction = 0.7;
  ga_obj.mutation_rate = 0.4;
  ga_obj.solve();

  output_file.close();
  fuzzer.Stop();
  return 0;
}
