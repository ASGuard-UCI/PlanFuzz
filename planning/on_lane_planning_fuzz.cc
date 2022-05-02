#include <stdlib.h>
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "text_format.h"
#include "binary_format.h"
#include "on_lane_planning_fuzz.h"
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
#include <dirent.h>

namespace apollo{
namespace planning{
namespace fuzzing{

using canbus::Chassis;
using perception::TrafficLightDetection;
using routing::RoutingResponse;
using prediction::PredictionObstacles;
using localization::LocalizationEstimate;
using common::time::Clock;
using tools::fuzz::planning::OnLanePlanningFuzzMessage;

OnLanePlanningFuzzer on_lane_planning_fuzzer;
bool OnLanePlanningFuzzer::FeedTestData(){
  //std::string fuzz_chassis_path = "/apollo/modules/planning/testdata/garage_test/garage_chassis.pb.txt";
  /*std::string fuzz_localization_path = "/apollo/modules/planning/testdata/garage_test/garage_localization.pb.txt";
  Chassis chassis;
  LocalizationEstimate localization_estimate;
  std::string fuzz_prediction_path = "/apollo/modules/planning/testdata/garage_test/garage_prediction.pb.txt";
  std::string fuzz_routing_path = "/apollo/modules/planning/testdata/garage_test/garage_routing.pb.txt";
  std::string fuzz_traffic_light_path = "";
  PredictionObstacles prediction_obstacles;
  RoutingResponse routing_response;
  TrafficLightDetection traffic_light_detection;

  if (!apollo::cyber::common::GetProtoFromFile(fuzz_chassis_path, &chassis))
    return false;
  std::cout<<"Finished parsing chassis"<<std::endl;
  if (!apollo::cyber::common::GetProtoFromFile(fuzz_localization_path, &localization_estimate)){
    std::cout<<"Failed to parse localization estimate"<<std::endl;
    return false;
  }
  std::cout<<"Finished parsing localization"<<std::endl;
  Clock::SetMode(Clock::MOCK);
  Clock::SetNowInSeconds(localization_estimate.header().timestamp_sec());
  if (!apollo::cyber::common::GetProtoFromFile(fuzz_prediction_path, &prediction_obstacles))
    return false;
  std::cout<<"Finished parsing prediction"<<std::endl;
  if (fuzz_traffic_light_path!="")
    if (!apollo::cyber::common::GetProtoFromFile(fuzz_traffic_light_path, &traffic_light_detection))
      return false;
  std::cout<<"Finished parsing prediction & traffic"<<std::endl;

  if (!apollo::cyber::common::GetProtoFromFile(fuzz_routing_path, &routing_response))
    return false;

  local_view_.prediction_obstacles =
    std::make_shared<PredictionObstacles>(prediction_obstacles);
  local_view_.chassis =
    std::make_shared<Chassis>(chassis);
  //local_view_.traffic_light =
  //  std::make_shared<TrafficLightDetection>(traffic_light_detection);
  local_view_.localization_estimate =
    std::make_shared<LocalizationEstimate>(localization_estimate);
  local_view_.routing =
    std::make_shared<RoutingResponse>(routing_response); */
  std::cout<<"Finished generate localview"<<std::endl;

}
void OnLanePlanningFuzzer::Warmup(){
  std::cout<<"Start of Warmup"<<std::endl;
  //Used for loading text msg
  /*
  OnLanePlanningFuzzMessage load_msg;
  int fd = open("/apollo/seeds/planning/crash", O_RDONLY);
  google::protobuf::io::FileInputStream raw_input(fd);
  google::protobuf::TextFormat::Parse(&raw_input, &load_msg);
  close(fd);
  std::ofstream output("/apollo/seeds/planning/binary_pb/crash", std::ios::out|std::ios::trunc|std::ios::binary);
  load_msg.SerializeToOstream(&output);
  output.close();*/
  //End of used for loading text ,sg
  FILE *f = fopen("/apollo/issue_2/data/input115", "rb");
  fseek(f, 0, SEEK_END);
  long fsize = ftell(f);
  fseek(f, 0, SEEK_SET);

  uint8_t *temp_buf = (uint8_t *)malloc(fsize + 1);
  fread(temp_buf, fsize, 1, f);
  fclose(f);

  temp_buf[fsize] = 0;
  RunPlanning(temp_buf ,fsize, "");
  sleep(2);
  std::cout<<"end of warm up"<<std::endl;
}

void OnLanePlanningFuzzer::GeneratePlanning(std::string path){
  int index=0; 
  DIR *dir;
  struct dirent *ptr;
  if ((dir=opendir(path.c_str()))==NULL){
    std::cout<<"Open DIR failed"<<std::endl;
    return;
  }
  while ((ptr = readdir(dir))!=NULL){
    if (strcmp(ptr->d_name, ".")==0 || strcmp(ptr->d_name, "..")==0)
      continue;
    else if (ptr->d_type == 8){
      std::string temp_str = std::string("/apollo/modules/tools/fuzz/vuln_test/data/") + std::string(ptr->d_name);
      std::ifstream in(temp_str, std::ios::binary);
      in.seekg(0, in.end);
      size_t length = in.tellg();
      in.seekg(0, in.beg);
      std::vector<char> bytes(length);
      in.read(bytes.data(), bytes.size());
      temp_str = temp_str + "_result";
      RunPlanning(reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size(),
		       temp_str);
    }
  }
}

void OnLanePlanningFuzzer::GetResult(OnLanePlanningFuzzMessage* input, ADCTrajectory* planning_traj){
  local_view.prediction_obstacles = std::make_shared<PredictionObstacles>();
  local_view.prediction_obstacles->CopyFrom(input->prediction_obstacles());
  local_view.chassis = std::make_shared<Chassis>();
  local_view.chassis->CopyFrom(input->chassis());
  local_view.localization_estimate = std::make_shared<LocalizationEstimate>();
  local_view.localization_estimate->CopyFrom(input->localization_estimate());
  local_view.routing = std::make_shared<RoutingResponse>();
  local_view.routing->CopyFrom(input->routing_response());
  local_view.traffic_light = std::make_shared<TrafficLightDetection>();
  if (input->has_traffic_light_detection()){
    local_view.traffic_light->CopyFrom(input->traffic_light_detection());
  }
  else
    AERROR<<"There is no traffic message";
  planning_->RunOnce(local_view, planning_traj);
  local_view.routing.reset();
  local_view.localization_estimate.reset();
  local_view.chassis.reset();
  local_view.prediction_obstacles.reset();
  local_view.traffic_light.reset();

}

bool OnLanePlanningFuzzer::RunPlanning(const uint8_t* data, size_t size, std::string save_path){
  std::cout<<"Start run planning"<<std::endl;
  ADCTrajectory adc_trajectory_pb;

  std::ofstream temp_out("temp_out.txt");
  temp_out<<"The size is "<<size<<std::endl;
  for (int i=0; i< size; i++)
    temp_out<<*(data+i);
  temp_out.close();



  bool result = on_lane_planning_fuzz_message.ParsePartialFromArray(data, size);
  //bool result = protobuf_mutator::ParseBinaryMessage(data, size, &on_lane_planning_fuzz_message);
  local_view.prediction_obstacles = std::make_shared<PredictionObstacles>();
  local_view.prediction_obstacles->CopyFrom(on_lane_planning_fuzz_message.prediction_obstacles());
  local_view.chassis = std::make_shared<Chassis>();
  local_view.chassis->CopyFrom(on_lane_planning_fuzz_message.chassis());
  local_view.localization_estimate = std::make_shared<LocalizationEstimate>();
  local_view.localization_estimate->CopyFrom(on_lane_planning_fuzz_message.localization_estimate());
  local_view.routing = std::make_shared<RoutingResponse>();
  local_view.routing->CopyFrom(on_lane_planning_fuzz_message.routing_response());
  //local_view.routing->mutable_road(1)->mutable_passage(0)->mutable_segment(0)->set_id("lane_1508");

  

  planning_->RunOnce(local_view, &adc_trajectory_pb);
  std::cout<<"Here is the end of Running Planning"<<std::endl;
  //planning_.reset();
  if (save_path!=""){
    std::fstream save_planning(save_path, std::ios::out | std::ios::trunc | std::ios::binary);
    adc_trajectory_pb.SerializeToOstream(&save_planning);
    //std::cout<<adc_trajectory_pb.DebugString()<<std::endl;
    save_planning.close();
  }
  local_view.routing.reset();
  local_view.localization_estimate.reset();
  local_view.chassis.reset();
  local_view.prediction_obstacles.reset();
  //planning_.reset();
  return false;
}

void OnLanePlanningFuzzer::Init(int argc, char **argv){
  ::apollo::cyber::Init("on_lane_planning_fuzz");
  //planning_ = std::unique_ptr<PlanningBase>(new OnLanePlanning());
  planning_.reset(new OnLanePlanning());

  FLAGS_enable_reference_line_provider_thread = false;
  FLAGS_planning_config_file = "/apollo/modules/planning/conf/planning_config.pb.txt";
  FLAGS_map_dir = std::string(getenv("APOLLO_MAP_PATH"));
  FLAGS_reckless_change_lane = true;
  if (!cyber::common::GetProtoFromFile(FLAGS_planning_config_file, &config_))
    std::cout<<"Failed to load config"<<std::endl;
  planning_->Init(config_);
  FLAGS_enable_nonscenario_side_pass = true;
}
void OnLanePlanningFuzzer::Stop(){
  planning_.reset();
}

} //end of namespace fuzzing
} //end of namespace planning
} //end of namespace apollo

int LLVMFuzzerInitialize(int *argc, char ***argv){
  std::cout<<"Start of LLVM Init"<<std::endl;
  apollo::planning::fuzzing::on_lane_planning_fuzzer.Init(*argc, *argv);
  //apollo::planning::fuzzing::on_lane_planning_fuzzer.Warmup();
  std::cout<<"End of LLVM Init"<<std::endl;
  return 0;
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size){
  //apollo::planning::fuzzing::on_lane_planning_fuzzer.GeneratePlanning("/apollo/modules/tools/fuzz/vuln_test/data");
  std::cout<<"LLVMFuzzerTestOneInput is involved"<<std::endl;
  try{
      apollo::planning::fuzzing::on_lane_planning_fuzzer.RunPlanning(data, size, "");
  } catch (int e) {
      return 0;
  }
  return 0;
}

int LLVMFuzzerStop(){
  apollo::planning::fuzzing::on_lane_planning_fuzzer.Stop();
  return 0;
}
