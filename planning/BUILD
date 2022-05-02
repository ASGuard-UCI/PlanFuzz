load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "on_lane_planning_fuzz_lib",
  srcs = [
      "on_lane_planning_fuzz.cc",
      "on_lane_planning_fuzz.h",
  ],
  copts = [
      "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
      "-lglog",
      "-liomp5",
      "-lpthread",
  ],
  deps = [
      "//modules/planning:planning_lib",
      "@glog",
      "@com_github_gflags_gflags//:gflags",
      "@libprotobuf_mutator//:mutator",
      "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],

)


cc_binary(
  name = "on_lane_planning_fuzz_bc",
  srcs = [
      "on_lane_planning_fuzz.cc",
      "afl/afl_driver.cpp",
      "afl/afl-llvm-rt.o.o",
      "on_lane_planning_fuzz.h",
  ],
  copts = [
      "-S",
      "-emit-llvm",
      "-fsanitize-coverage=trace-pc-guard",
      "-ftest-coverage",
      "-fprofile-arcs",
      "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
      "-S",
      "-emit-llvm",
      "-fsanitize-coverage=trace-pc-guard",
      "--coverage",
      "-lglog",
      "-liomp5",
      "-lpthread",
  ],
  deps = [
      "//modules/planning:planning_lib",
      "@glog",
      "@com_github_gflags_gflags//:gflags",
      "@libprotobuf_mutator//:mutator",
      "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],

)

cc_binary(
  name = "on_lane_planning_fuzz",
  srcs = [
      "on_lane_planning_fuzz.cc",
      "afl/afl_driver.cpp",
      "afl/afl-llvm-rt.o.o",
      "on_lane_planning_fuzz.h",
  ],
  copts = [
      "-save-temps",
      #"-fsanitize-coverage=trace-pc-guard",
      #"-ftest-coverage",
      "-fprofile-arcs",
      "-Iexternal/libprotobuf_mutator/src",
  ],
  linkopts = [
      #"-fsanitize-coverage=trace-pc-guard",
      "-lglog",
      "-liomp5",
      "-lpthread",
  ],
  deps = [
      "//modules/planning:planning_lib",
      "@glog",
      "@com_github_gflags_gflags//:gflags",
      "@libprotobuf_mutator//:mutator",
      "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],

)
cc_binary(
  name = "on_lane_planning_fuzz_mutator.so",
  srcs = [
    "mutator.cc",
  ],
  deps = [
    "@libprotobuf_mutator//:mutator",
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],
  linkshared = 1,
  linkstatic = 1,
)
cc_binary(
  name="on_lane_planning_msg_parser",
  srcs = [
    "msg_parser/msg_parser.cc",
  ],
  deps = [
    "//modules/tools/fuzz/planning/proto:on_lane_planning_msg_proto",
  ],
)

cpplint()
