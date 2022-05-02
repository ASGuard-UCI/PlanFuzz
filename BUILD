load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "execute_planning",
  srcs = [
    "tools/execute_planning.cc",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/vuln_test:fit_tracker_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
    "//modules/tools/fuzz/vuln_test/planning_context:planning_context_util_lib",
  ],
)

cc_binary(
  name = "openga_test",
  srcs = [
    "mutation/planning_input_mutator.cc",
    "mutation/planning_input_mutator.h",
    "mutation/checker/lane_checker.cc",
    "mutation/checker/lane_checker.h",
    "opt/openGA/so.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)



cc_binary(
  name = "evolutionary_test",
  srcs = [
    "mutation/planning_input_mutator.cc",
    "mutation/planning_input_mutator.h",
    "mutation/checker/lane_checker.cc",
    "mutation/checker/lane_checker.h",
    "opt/evolver.cc",
    "opt/evolver.h",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
    "tools/fitness_tracker.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//modules/tools/fuzz/vuln_test:fit_tracker_lib",
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)


cc_binary(
  name = "mutation_test",
  srcs = [
    "mutation/planning_input_mutator.cc",
    "mutation/planning_input_mutator.h",
    "mutation/checker/lane_checker.cc",
    "mutation/checker/lane_checker.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],
)


cc_library(
  name = "fit_tracker_lib",
  srcs = [
    "tools/fitness_tracker.cc",
  ],
  hdrs = [
    "tools/fitness_tracker.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//modules/common/math",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],
)

cc_binary(
  name = "lane_change_test",
  srcs = [
    "mutation/planning_input_mutator.cc",
    "mutation/planning_input_mutator.h",
    "mutation/checker/lane_checker.cc",
    "mutation/checker/lane_checker.h",
    "mutation/map_util/map_util.h",
    "mutation/map_util/map_util.cc",
    "scenario/lane_change/openGA/so.cc",
    "scenario/lane_change/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)

cc_binary(
  name = "crash_test",
  srcs = [
    "mutation/planning_input_mutator.cc",
    "mutation/planning_input_mutator.h",
    "mutation/checker/lane_checker.cc",
    "mutation/checker/lane_checker.h",
    "mutation/map_util/map_util.h",
    "mutation/map_util/map_util.cc",
    "scenario/crash/openGA/so.cc",
    "scenario/crash/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)


cc_binary(
  name = "lane_change_test_v1",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/lane_change/openGA/so_v1.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)

cc_binary(
  name = "single_test",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/lane_borrow/single_test.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)

cc_binary(
  name = "lane_borrow_test",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/lane_borrow/openGA/so.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
  ],
)


cc_binary(
  name = "sequence_play",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/intersection/sequence_play.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
    "//modules/tools/fuzz/vuln_test/planning_context:planning_context_util_lib",
  ],
)

cc_binary(
  name = "intersection_test",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/intersection/openGA/so.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
    "//modules/tools/fuzz/vuln_test/planning_context:planning_context_util_lib",
  ],
)

cc_binary(
  name = "intersection_random_search",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/intersection/openGA/random_search.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
    "//modules/tools/fuzz/vuln_test/planning_context:planning_context_util_lib",
  ],
)

cc_binary(
  name = "intersection_pbm",
  srcs = [
    #"mutation/planning_input_mutator.cc",
    #"mutation/planning_input_mutator.h",
    #"mutation/checker/lane_checker.cc",
    #"mutation/checker/lane_checker.h",
    "mutation_v1/mutate_planning.cc",
    "mutation_v1/mutate_planning.h",
    "mutation_v1/map_util/map_util.h",
    "mutation_v1/map_util/map_util.cc",
    "mutation_v1/mutation_util.cc",
    "mutation_v1/mutation_util.h",
    "scenario/intersection/openGA/protobuf.cc",
    "opt/openGA/openGA.hpp",
    "fitness/traj_based.cc", 
    "fitness/fitness.h",
  ],
  copts = [
    "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
    "-lpthread",
  ],
  deps = [
    "//cyber",
    "//modules/common/math",
    "@com_github_gflags_gflags//:gflags",
    "//modules/map/hdmap:hdmap_util",
    "@libprotobuf_mutator//:mutator",
    "//modules/planning:planning_lib",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
    "//modules/tools/fuzz/planning:on_lane_planning_fuzz_lib",
    "//modules/tools/fuzz/vuln_test/planning_context:planning_context_util_lib",
  ],
)


