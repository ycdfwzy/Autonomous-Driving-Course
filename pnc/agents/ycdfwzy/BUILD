package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ycdfwzy_agent",
    srcs = ["ycdfwzy_agent.cc"],
    hdrs = ["ycdfwzy_agent.h"],
    deps = [
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//common/utils/file",
        "//pnc/map:map_lib",
        "//common/proto:cc_route_proto",
        "//common/proto:cc_geometry_proto",
        "//common/proto:cc_transform_proto",
        "@glog",
        "@gflags//:gflags",
        #":FindRoute",
        "@eigen//:eigen",
    ],
)
