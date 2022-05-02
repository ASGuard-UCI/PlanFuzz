#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
#include <stdio.h>
#include "modules/tools/fuzz/planning/on_lane_planning_fuzz.h"
#include "modules/planning/proto/planning.pb.h"
#include "fitness_tracker.h"




using apollo::planning::ADCTrajectory;
using apollo::tools::fuzz::planning::OnLanePlanningFuzzMessage;
using apollo::planning::fuzzing::OnLanePlanningFuzzer;
OnLanePlanningFuzzer fuzzer;
FitnessTracker fitness_tracker;
void executePlanning(int index){

  char input_path[100]; 
  char output_path[100];
  sprintf(output_path, "/apollo/issue_2/data/local_planningout%d", index);
  
  sprintf(input_path, "/apollo/issue_2/data/input%d", index);
  std::ifstream input(input_path, std::ios::in | std::ios::binary);
  OnLanePlanningFuzzMessage msg;
  msg.ParsePartialFromIstream(&input);
  msg.mutable_routing_response()->mutable_road(1)->mutable_passage(0)->mutable_segment(0)->set_id("lane_1508");
  AERROR<<msg.routing_response().road(1).passage(0).segment(0).DebugString();  
  ADCTrajectory temp;
  fuzzer.GetResult(&msg, &temp);
  std::ofstream output(output_path, std::ios::out | std::ios::trunc | std::ios::binary);
  temp.SerializeToOstream(&output);
  
  std::stringstream ss;
  ss<<"/apollo/issue_2/data/debug"<<index;
  std::ofstream debug_output(ss.str(), std::ofstream::app);
  debug_output<<temp.DebugString();
  debug_output.close();
  output.close(); 
  return;
}



void executePlanningArgv(char** argv){

  char input_path[100]; 
  char output_path[100];
  sprintf(output_path, argv[1]);
  
  sprintf(input_path, argv[2]);
  AERROR<<input_path;
  std::ifstream input(input_path, std::ios::in | std::ios::binary);
  OnLanePlanningFuzzMessage msg;
  msg.ParsePartialFromIstream(&input);
//  AERROR<<"The input msg is \n"<<msg.DebugString();
  ADCTrajectory temp;
  fuzzer.GetResult(&msg, &temp);
  std::ofstream output(output_path, std::ios::out | std::ios::trunc | std::ios::binary);
  temp.SerializeToOstream(&output);
  
  std::stringstream ss;
  //ss<<"/apollo/issue_2/data/debug"<<index;
  std::ofstream debug_output(ss.str(), std::ofstream::app);
  //debug_output<<temp.DebugString();
  //debug_output.close();
  output.close(); 
  return;
}



int main(int argc, char**  argv){
  fuzzer.Init(argc, argv); 
//  fuzzer.Warmup();

  std::ofstream output("exe_fit.txt", std::ios::trunc);
  std::ofstream logout("log_fit.txt", std::ios::trunc);
  if (getenv("INST_CONFIG_PATH")!=NULL)
    LoadInstInfo(std::string(getenv("INST_CONFIG_PATH")));
  else
    AERROR<<"Can not find env variable INST_CONFIG_PATH";
 
  if (getenv("APOLLO_MAP_PATH")!=NULL)
    FLAGS_map_dir = getenv("APOLLO_MAP_PATH");
  else
    AERROR<<"Can not find env variable APOLLO_MAP__PATH";
   

  //for (int i=10; i<136; i++){
    init_shared_mem();
    fitness_tracker.Init(&logout);
    executePlanningArgv(argv);
    executePlanningArgv(argv);
    double exec_fitness = get_fitness_shared_mem(); 
    //output<<i<<" "<<fitness_tracker.GetFitness()<<std::endl;
    output<<" exec fitness is"<<exec_fitness<<std::endl;
    //for (int j=0; j< 6; j++)
    //  output<<shared_mem[j]<<" ";
    //for (int j=0; j<6; j++)
    //  output<<shared_mem1[j]<<" ";
    //output<<"\n";
    std::cout<<"The debug flag is "<<debug_flag<<'\n';
  //}
  fuzzer.Stop();
  output.close();
  logout.close();
  std::cout<<"End of progeam !\n";
  return 0;
}

