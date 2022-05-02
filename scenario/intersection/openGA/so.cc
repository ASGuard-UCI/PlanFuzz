#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include "modules/tools/fuzz/vuln_test/opt/openGA/openGA.hpp"
#include "modules/tools/fuzz/vuln_test/planning_context/planning_context_util.h"
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
apollo::prediction::PredictionObstacle stored_pred_obs;
std::string init_seed_path;
std::string benign_traj_path;
ADCTrajectory benign_traj;
std::string input_debug_path = "/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/data/inputdebug";
std::string input_msg_path;
std::string output_debug_path = "/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/data/outputdebug";
std::string output_traj_path;
int max_index = 99999999;
PlanningContextPb prev_context;
std::string context_path;
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
GA_Type ga_obj_nodata;
GA_Type ga_obj_noPI;
bool enable_PI=0;
int enable_data=1;
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
 // int fd = open("/apollo/seeds/planning/straight_traffic_light", O_RDONLY);
 // google::protobuf::io::FileInputStream raw_input(fd);
 // google::protobuf::TextFormat::Parse(&raw_input, &(p.input));

  //mutator.GenerateInitialData(&(p.input));
  std::ifstream input(init_seed_path, std::ios::binary|std::ios::in);
  AERROR<<init_seed_path;
  p.input.ParseFromIstream(&input);
  if (p.input.prediction_obstacles().prediction_obstacle_size()>0){
    stored_pred_obs.CopyFrom(p.input.prediction_obstacles().prediction_obstacle(0));
  }
  //std::cout<<"begin print pb message"<<std::endl;
  //std::cout<<p.input.DebugString()<<std::endl;
  
  mutator.GenerateInitialData(&(p.input), enable_PI);
  //close(fd);
  //std::ofstream output("/apollo/modules/tools/fuzz/vuln_test/scenario/crash/data/planningmsg0", std::ios::out|std::ios::trunc|std::ios::binary);
  //p.input.SerializeToOstream(&output);
  //output.close();
}


bool check_vuln(ADCTrajectory *temp){
  if (benign_traj.decision().main_decision().has_stop() && temp->decision().main_decision().has_stop()){
    if (benign_traj.decision().main_decision().stop().reason_code() != temp->decision().main_decision().stop().reason_code()){
      AERROR<<benign_traj.decision().DebugString();
      AERROR<<temp->decision().DebugString();
     AERROR<<"Program stop because of finding a vuln (planning context is different)\n"; 
     std::cout<<"Program stop because of finding a vuln and stop reason is different\n"; 
      std::cout<<"The benign and under-attack stop reason code is "<<benign_traj.decision().main_decision().stop().reason_code()<<" "<<temp->decision().main_decision().stop().reason_code()<<std::endl;
      ga_obj.user_request_stop = true;
      return true;
     }
      if (benign_traj.decision().main_decision().stop().change_lane_type() != temp->decision().main_decision().stop().change_lane_type()){
      AERROR<<benign_traj.decision().DebugString();
      AERROR<<temp->decision().DebugString();
     AERROR<<"Program stop because of finding a vuln (planning context is different)\n"; 
     std::cout<<"Program stop because of finding a vuln and change lane type is different\n"; 
      ga_obj.user_request_stop = true;
      return true;
     }

  } 
  else if (temp->decision().main_decision().has_stop()){
    AERROR<<"Program stop because of finding a vuln (planning context is different)\n"; 
    std::cout<<"Program stop because of finding a vuln (The original decision is not stop)\n"; 
    AERROR<<benign_traj.decision().DebugString();
    AERROR<<temp->decision().DebugString();
      ga_obj.user_request_stop = true;
    return true;
  }
  else if (benign_traj.decision().main_decision().has_stop()){
    if (benign_traj.decision().main_decision().stop().reason_code() == 2){
    AERROR<<"Program stop because of finding a vuln (planning context is different)\n"; 
    std::cout<<"Program stop because of finding a vuln (benign stop reason code is 2)\n"; 
    AERROR<<benign_traj.decision().DebugString();
    AERROR<<temp->decision().DebugString();
      ga_obj.user_request_stop = true;
    return true;

    }
  }
  PlanningContextPb temp_context;
  StorePlanningContext(&temp_context);
  if (CompareContext(temp_context, prev_context) == false){
    AERROR<<"Program stop because of finding a vuln (planning context is different)\n"; 
    std::cout<<"Program stop because of finding a vuln (planning context is different)\n"; 
    AERROR<<benign_traj.decision().DebugString();
    AERROR<<temp->decision().DebugString();
    ga_obj.user_request_stop = true;
    return true;
  }
  return false;
}

bool eval_solution(const MySolution& p, MyMiddleCost &c){
  ADCTrajectory* temp = new ADCTrajectory;
  refresh_shared_mem();
  //fitness_tracker.Init(log_pointer);
  OnLanePlanningFuzzMessage tmp_msg;
    tmp_msg.CopyFrom(p.input);
  PredictionObstacle* temp_pred_obs = tmp_msg.mutable_prediction_obstacles()->add_prediction_obstacle();
  temp_pred_obs->CopyFrom(stored_pred_obs);
  LoadPlanningContext(context_path);
  StorePlanningContext(&prev_context);
  /*for (int i=0; i< tmp_msg.prediction_obstacles().prediction_obstacle_size(); i++){
    tmp_msg.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle()->set_theta(0.0);
     tmp_msg.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle()->mutable_velocity()->set_y(0.0);
     tmp_msg.mutable_prediction_obstacles()->mutable_prediction_obstacle(i)->mutable_perception_obstacle()->mutable_velocity()->set_x(0.0);
    
  }*/
  //apollo::planning::PlanningContext::Instance()->mutable_planning_status()->CopyFrom(stored_planning_status);
  //apollo::planning::PlanningContext::Instance()->mutable_path_decider_info()->CopyFrom(stored_path_decider_info);
  //std::cout<<stored_path_decider_info.DebugString();
  AERROR<<"Start executing case "<<case_index<<'\n';
  fuzzer.GetResult(&tmp_msg, temp);
  std::cout<<"Getting fitness value for "<<case_index<<'\n';
  if (enable_data)
    c.fitness = get_fitness_shared_mem();
  else
    c.fitness = get_fitness_shared_mem_AFLGO();
 
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
  //if (temp->decision().main_decision().has_stop()){
  //  if (temp->decision().main_decision().stop().reason_code()!=2){
  //    std::cout<<"Program stop because of finding a vuln!\n";
  //    ga_obj.user_request_stop = true;
  //  }
  //}
  //AERROR<<benign_traj.decision().DebugString();
  //AERROR<<temp->decision().DebugString();
  if (!enable_PI){
    if (mutator.VerifyPICons(tmp_msg) == false){
      case_index++;
      ga_obj.user_request_stop=false;
      return true;
    }
  }

  if (benign_traj.decision().main_decision().has_stop() && temp->decision().main_decision().has_stop()){
    if (benign_traj.decision().main_decision().stop().reason_code() != temp->decision().main_decision().stop().reason_code()){
      AERROR<<benign_traj.decision().DebugString();
      AERROR<<temp->decision().DebugString();
      std::cout<<"Program stop because of finding a vuln and stop reason is different\n"; 
      std::cout<<"The benign and under-attack stop reason code is "<<benign_traj.decision().main_decision().stop().reason_code()<<" "<<temp->decision().main_decision().stop().reason_code()<<std::endl;
      if (enable_PI && enable_data) ga_obj.user_request_stop = true;
      else if (enable_PI==0 && enable_data==1) ga_obj_noPI.user_request_stop=true;
      else if (enable_PI==1 && enable_data==0) ga_obj_nodata.user_request_stop=true;
    
     }
      if (benign_traj.decision().main_decision().stop().change_lane_type() != temp->decision().main_decision().stop().change_lane_type()){
      AERROR<<benign_traj.decision().DebugString();
      AERROR<<temp->decision().DebugString();
      std::cout<<"Program stop because of finding a vuln and change lane type is different\n"; 
      if (enable_PI && enable_data) ga_obj.user_request_stop = true;
      else if (enable_PI==0 && enable_data==1) ga_obj_noPI.user_request_stop=true;
      else if (enable_PI==1 && enable_data==0) ga_obj_nodata.user_request_stop=true;
    

     }

  } 
  else if (temp->decision().main_decision().has_stop()){
    std::cout<<"Program stop because of finding a vuln (The original decision is not stop)\n"; 
    AERROR<<benign_traj.decision().DebugString();
    AERROR<<temp->decision().DebugString();
    if (enable_PI && enable_data) ga_obj.user_request_stop = true;
      else if (enable_PI==0 && enable_data==1) ga_obj_noPI.user_request_stop=true;
      else if (enable_PI==1 && enable_data==0) ga_obj_nodata.user_request_stop=true;
    

   
  }
  else if (benign_traj.decision().main_decision().has_stop()){
    if (benign_traj.decision().main_decision().stop().reason_code() == 2){
    std::cout<<"Program stop because of finding a vuln (benign stop reason code is 2)\n"; 
    AERROR<<benign_traj.decision().DebugString();
    AERROR<<temp->decision().DebugString();
    if (enable_PI && enable_data) ga_obj.user_request_stop = true;
      else if (enable_PI==0 && enable_data==1) ga_obj_noPI.user_request_stop=true;
      else if (enable_PI==1 && enable_data==0) ga_obj_nodata.user_request_stop=true;
    

   

    }
  }
  PlanningContextPb temp_context;
  StorePlanningContext(&temp_context);
  if (CompareContext(temp_context, prev_context) == false){
    std::cout<<"Program stop because of finding a vuln (planning context is different)\n"; 
    AERROR<<benign_traj.decision().DebugString();
    AERROR<<temp->decision().DebugString();
    if (enable_PI && enable_data) ga_obj.user_request_stop = true;
      else if (enable_PI==0 && enable_data==1) ga_obj_noPI.user_request_stop=true;
      else if (enable_PI==1 && enable_data==0) ga_obj_nodata.user_request_stop=true;
    

  }

  if (case_index == max_index){
    ga_obj.user_request_stop = true;
    std::cout<<"Reach max exec number"<<std::endl;
  }
  case_index++;
  return true;
}



MySolution crossover(const MySolution& X1, const MySolution&X2, const std::function<double(void)> &rnd01){
  MySolution temp_s1, temp_s2;
  temp_s1.input.CopyFrom(X1.input);
  temp_s2.input.CopyFrom(X2.input);
  mutator.CrossoverPlanningMsg(&temp_s1.input, &temp_s2.input);
  return temp_s1; 
}

MySolution mutate(const MySolution& X_base, const std::function<double(void)> &rnd01, double shrink_scale){
  MySolution X_ret;
  X_ret.input.CopyFrom(X_base.input);
  mutator.MutatePlanningMsgTrans(&X_ret.input, enable_PI);
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

  std::stringstream ss;
  ss<<"pos_log/pos_log_"<<generation_number;
  std::ofstream pos_out(ss.str(), std::ios::out|std::ios::trunc);
  for (auto c: last_generation.chromosomes){
    for (int j=0; j<2; j++){
      pos_out<<std::setprecision(10)
	     <<c.genes.input.prediction_obstacles().prediction_obstacle(j).perception_obstacle().position().x()<<" "
  	     <<c.genes.input.prediction_obstacles().prediction_obstacle(j).perception_obstacle().position().y()<<" ";
      if (c.genes.input.prediction_obstacles().prediction_obstacle(j).perception_obstacle().type()  == apollo::perception::PerceptionObstacle::VEHICLE)
        pos_out<<"V\n";
       if (c.genes.input.prediction_obstacles().prediction_obstacle(j).perception_obstacle().type()  == apollo::perception::PerceptionObstacle::UNKNOWN_UNMOVABLE)
        pos_out<<"S\n";
       if (c.genes.input.prediction_obstacles().prediction_obstacle(j).perception_obstacle().type()  == apollo::perception::PerceptionObstacle::PEDESTRIAN)
        pos_out<<"P\n";
      
    }
  }
  pos_out.close();
}

int main(int argc, char** argv){
  FLAGS_map_dir = "/apollo/modules/map/data/single_lane_road";
  output_file.open("./result_so1.txt");
  init_seed_path = std::string(argv[1]);
  benign_traj_path = std::string(argv[2]);
  input_msg_path = std::string(argv[3]);
  output_traj_path = std::string(argv[4]);
  context_path = std::string(argv[5]);
  AERROR<<"Init the input and output path "<<input_msg_path<<" "<<output_traj_path;
  std::ofstream log_file("log_fit.txt", std::ios::out);
  log_pointer = &log_file;
  
  std::cout<<"Beginning of OpenGA process"<<std::endl;
  fuzzer.Init(argc, argv);
  //fuzzer.Warmup();
  std::cout<<"Finisht the warmup of fuzzer"<<std::endl;
 
  std::ifstream init_seed_if(init_seed_path, std::ios::in|std::ios::binary);
  std::ifstream benign_traj_if(benign_traj_path, std::ios::in|std::ios::binary);
  OnLanePlanningFuzzMessage init_seed; 
  init_seed.ParseFromIstream(&init_seed_if);
  benign_traj.ParseFromIstream(&benign_traj_if); 
  mutator.LoadBlackList(&init_seed, &benign_traj);
  //copy_context();
  //mutator.LoadBlackList("/apollo/modules/tools/fuzz/vuln_test/scenario/lane_borrow/lane_borrow_blacklist");
  fitness_tracker.Init(&log_file);
  init_shared_mem();
  EA::Chronometer timer;
  timer.tic();
  LoadInstInfo(std::string(getenv("INST_CONFIG_PATH")));
  //GA_Type ga_obj;
  std::string pi_flag(getenv("ENABLE_PI")); 
  if (pi_flag == "True" || pi_flag == "true"){
    enable_PI = true;
    std::cout<<"Enable_PI flag is true";
  }
  else{
    enable_PI = false;
    std::cout<<"Enable_PI flag is false";
  }
  ga_obj.problem_mode = EA::GA_MODE::SOGA;
  ga_obj.multi_threading=false;
  ga_obj.idle_delay_us = 1;
  ga_obj.verbose =true;
  ga_obj.population = 50;
  ga_obj.generation_max = 300;
  ga_obj.calculate_SO_total_fitness = calculate_SO_total_fitness;
  ga_obj.init_genes = init_genes;
  ga_obj.eval_solution = eval_solution;
  ga_obj.mutate = mutate;
  ga_obj.crossover = crossover;
  ga_obj.SO_report_generation = SO_report_generation;
  ga_obj.best_stall_max = 200;
  ga_obj.average_stall_max = 200;
  ga_obj.elite_count = 15;
  ga_obj.crossover_fraction = 0.7;
  ga_obj.mutation_rate = 0.4;
  ga_obj.solve();
/*  max_index = case_index;
  std::cout<<"Max index is "<<max_index<<std::endl;
  enable_data = 0;
  fitness_tracker.Init(&log_file);
  init_shared_mem();
  LoadInstInfo(std::string(getenv("INST_CONFIG_PATH")));
  //GA_Type ga_obj;
  case_index=0;
  enable_data=1;
  enable_PI=0; 
  ga_obj_nodata.problem_mode = EA::GA_MODE::SOGA;
  ga_obj_nodata.multi_threading=false;
  ga_obj_nodata.idle_delay_us = 1;
  ga_obj_nodata.verbose =true;
  ga_obj_nodata.population = 50;
  ga_obj_nodata.generation_max = 300;
  ga_obj_nodata.calculate_SO_total_fitness = calculate_SO_total_fitness;
  ga_obj_nodata.init_genes = init_genes;
  ga_obj_nodata.eval_solution = eval_solution;
  ga_obj_nodata.mutate = mutate;
  ga_obj_nodata.crossover = crossover;
  ga_obj_nodata.SO_report_generation = SO_report_generation;
  ga_obj_nodata.best_stall_max = 200;
  ga_obj_nodata.average_stall_max = 200;
  ga_obj_nodata.elite_count = 15;
  ga_obj_nodata.crossover_fraction = 0.7;
  ga_obj_nodata.mutation_rate = 0.4;
  ga_obj_nodata.solve();


  init_shared_mem();
  LoadInstInfo(std::string(getenv("INST_CONFIG_PATH")));
  //GA_Type ga_obj;
  case_index=0;
  ga_obj_noPI.problem_mode = EA::GA_MODE::SOGA;
  ga_obj_noPI.multi_threading=false;
  ga_obj_noPI.idle_delay_us = 1;
  ga_obj_noPI.verbose =true;
  ga_obj_noPI.population = 50;
  ga_obj_noPI.generation_max = 300;
  ga_obj_noPI.calculate_SO_total_fitness = calculate_SO_total_fitness;
  ga_obj_noPI.init_genes = init_genes;
  ga_obj_noPI.eval_solution = eval_solution;
  ga_obj_noPI.mutate = mutate;
  ga_obj_noPI.crossover = crossover;
  ga_obj_noPI.SO_report_generation = SO_report_generation;
  ga_obj_noPI.best_stall_max = 200;
  ga_obj_noPI.average_stall_max = 200;
  ga_obj_noPI.elite_count = 15;
  ga_obj_noPI.crossover_fraction = 0.7;
  ga_obj_noPI.mutation_rate = 0.4;
  ga_obj_noPI.solve();*/


  output_file.close();
  fuzzer.Stop();
  return 0;
}
