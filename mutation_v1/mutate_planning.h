#include <vector>
#include <iostream>
#include <string>
#include "modules/tools/fuzz/planning/proto/on_lane_planning_msg.pb.h"
#include "map_util/map_util.h"
#include "modules/planning/proto/planning.pb.h"


using apollo::localization::Pose;
using apollo::common::Point3D;
using apollo::perception::PerceptionObstacle;
using apollo::tools::fuzz::planning::OnLanePlanningFuzzMessage;
using apollo::prediction::PredictionObstacle;
using apollo::planning::ADCTrajectory;

typedef struct BlackAreaStr{
  double min_s, min_l, max_s, max_l;;
  std::string lane_id;
} BlackArea;


class PlanningMutator {
  std::vector<BlackArea> black_list;
	public:
  bool CheckBBoxOnRoad(PerceptionObstacle *input);
  void LoadBlackList(std::string config_path);
  std::vector<Point3D> GetCorners(const PerceptionObstacle *input);
  bool CheckPoint(Point3D pos);
  void InitPedestrian(PerceptionObstacle *x, double center_x, double center_y, int enable_PI=1);
  void InitVehicle(PerceptionObstacle *x, double center_x, double center_y, int enable_PI=1);
  void InitStatic(PerceptionObstacle *x, double center_x, double center_y, int enable_PI=1);
  apollo::prediction::Trajectory GenerateTrajectory(const std::vector<apollo::common::TrajectoryPoint>&);
  bool VerifyPICons(PredictionObstacle *input);
  bool VerifyPICons(OnLanePlanningFuzzMessage input);
  void LoadBlackList(OnLanePlanningFuzzMessage *input, ADCTrajectory *output);
  bool CheckBBox(PerceptionObstacle* input);
  void MutatePosHeading(PerceptionObstacle *input);
  void MutatePlanningMsg(OnLanePlanningFuzzMessage *input, int enable_PI=1);
  void MutatePlanningMsgTrans(OnLanePlanningFuzzMessage *input);
  void MutatePredictionObstacle(PredictionObstacle *input, int enable_PI=1);
  void DrawTrajectory(PredictionObstacle *input);
  bool VerifyTrajectory(PredictionObstacle *input);
  void MutatePredictionObstacleTrans(PredictionObstacle *input);
  //void CrossoverPredictionObstacle(PredictionObstacle *input1, PredictionObstacle *input2);
  //void CrossoverPerceptionObstacle(PerceptionObstacle *input1, PerceptionObstacle *input2);
  void MutatePerceptionObstacle(PerceptionObstacle *input, int enable_PI=1);
  void MutatePedestrian(PerceptionObstacle *input, int enable_PI=1);
  void MutateVehicle(PerceptionObstacle *input, int enable_PI=1);
  void MutateStatic(PerceptionObstacle *input, int enable_PI=1);
  void MutatePerceptionObstacleTrans(PerceptionObstacle *input);
  void CrossoverPlanningMsg(OnLanePlanningFuzzMessage *input1, OnLanePlanningFuzzMessage *input2);
  void GenerateInitialData(OnLanePlanningFuzzMessage *input, int enable_PI=1); 
  double BarrierFunc(OnLanePlanningFuzzMessage *input);
  double OnLaneBarrierFunc(OnLanePlanningFuzzMessage *input);
  void GetMinDistFromBound(const PerceptionObstacle *input, double *barrier_s, double *barrier_l);
  double GetBarrierValue(double x, double y, double con1, double con2);
  void TransportOffRd(PerceptionObstacle *input);
  void Transport(Point3D *pos, int obs_idx, int direction, double buffer_s, double buffer_l);
  std::vector<Point3D> Transport(const Point3D, const Point3D, double, double);
  int CheckPointWithBuffer(const Point3D, double, double);
  int InferDirection(const Point3D, const Point3D, int, double, double);
};

