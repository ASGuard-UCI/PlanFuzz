#ifndef __ON_LANE_PLANNING_FUZZER__
#define __ON_LANE_PLANNING_FUZZER__

#include <memory>
#include <string>

#include "modules/planning/on_lane_planning.h"
#include "modules/planning/planning_base.h"
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"

namespace apollo{
namespace planning{
namespace fuzzing{
class OnLanePlanningFuzzer{
  public:
      bool FeedTestData();
      bool RunPlanning(const uint8_t* data, size_t size, std::string save_path);
      void Init(int argc, char **argv);
      void Stop();
      void Warmup();
      void GeneratePlanning(std::string path);
      void GetResult(tools::fuzz::planning::OnLanePlanningFuzzMessage* input, ADCTrajectory* planning_traj);
  protected:
    std::unique_ptr<PlanningBase> planning_ =nullptr;
    std::map<TrafficRuleConfig::RuleId, bool> rule_enabled_;
    PlanningConfig config_;
    LocalView local_view;
    tools::fuzz::planning::OnLanePlanningFuzzMessage on_lane_planning_fuzz_message;

} ;

} //end of namespace fuzzing
} //end of namespace planning
} //end of namespace apollo

int LLVMFuzzerInitialize(int *argc, char ***argv);
int LLVMFuzzerTestOneInput(const uint8_t*, size_t size);
int LLVMFuzzerStop();

#endif
