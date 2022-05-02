#include <iostream>
#include <fstream>
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
int main(int argc, char** argv){
  if (argc==2){
    apollo::tools::fuzz::planning::OnLanePlanningFuzzMessage msg;
    std::fstream input(*(argv+1), std::ios::in | std::ios::binary);
    msg.ParsePartialFromIstream(&input);
    std::string out_string;
    std::cout<<msg.DebugString();
    input.close();
  }
  return 0;
}
