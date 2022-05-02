import time
import sys

from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.prediction.proto.prediction_obstacle_pb2 import PredicxtionObstacles
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.tools.fuzz.on_lane_planning_msg_pb2 import OnLanePlanningFuzzMessage

if __name__ == '__main__':
  init_input = OnLanePlanningFuzzMessage()
  f = open("/apollo/seed/planning/binary_pb/lane_follow", "rb")
  init_input.ParseFromString(f.read()) 
  f.close()
  init_input.prediction.prediction_obstacles.clear()
  print(init_input)

