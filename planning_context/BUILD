load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_proto_library(
  name = "planning_context_proto",
  deps = [
    "planning_context_proto_lib",
  ],
)


proto_library(
  name = "planning_context_proto_lib",
  srcs = [
    "planning_context.proto",
  ], 
  deps = [
    "//modules/planning/proto:planning_status_proto_lib",
    "//modules/planning/proto:path_decider_info_proto_lib",
  ], 

)

cc_library(
  name = "planning_context_util_lib",
  srcs = [
    "planning_context_util.cc",
  ],
  hdrs = [
    "planning_context_util.h",
  ],
  deps = [
    "//modules/planning/common:planning_context",
    "planning_context_proto",
    "//cyber/common:log",
    "//modules/map/pnc_map:path",
    "//modules/planning/common/path:path_data",
    "//modules/planning/proto:path_decider_info_proto",
    "//modules/planning/proto:planning_config_proto",
     "//modules/planning/proto:planning_proto",
     "//modules/planning/proto:planning_status_proto",
     "@eigen",
     "//modules/planning/common:planning_gflags",
  ], 
)
