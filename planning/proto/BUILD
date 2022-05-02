package(default_visibility = ["//visibility:public"])

cc_proto_library(
  name = "on_lane_planning_msg_proto",
  deps = [
    ":on_lane_planning_msg_proto_lib",
  ],
)

cc_binary(
  name = "on_lane_planning_fuzz_msg_shared.so",
  deps = [
    ":on_lane_planning_msg_proto",
  ],
  linkopts = [
  #  '-static',
  ],
  linkshared = 1,
  linkstatic = 1,
)


proto_library(
  name = "on_lane_planning_msg_proto_lib",
  srcs = [
      "on_lane_planning_msg.proto",
  ],
  deps = [
      "//modules/localization/proto:localization_proto_lib",
      "//modules/canbus/proto:canbus_proto_lib",
      "//modules/routing/proto:routing_proto_lib",
      "//modules/prediction/proto:prediction_proto_lib",
      "//modules/perception/proto:perception_proto_lib",
  ],
)
