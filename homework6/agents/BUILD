package(default_visibility = ["//visibility:public"])

cc_library(
    name = "agents",
    hdrs = ["agents.h"],
    srcs = ["agents.cc"],
    deps = [
        "//homework6/agents/sample:sample_agent",
        "//homework6/simulation:vehicle_agent_factory",
        ":straightagent",
        ":turnsagent",
    ],
    alwayslink = 1,
)

cc_library(
	name = "straightagent",
	hdrs = ["straight_agent.h"],
	srcs = ["straight_agent.cc"],
	deps = [
		"//homework6/simulation:vehicle_agent",
        "//homework6/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
	],
)

cc_library(
    name = "turnsagent",
    hdrs = ["turns_agent.h"],
    srcs = ["turns_agent.cc"],
    deps = [
        "//homework6/simulation:vehicle_agent",
        "//homework6/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//homework5:route",
        "//homework5:predecessors_and_successors",
        "//homework6/map:map_lib",
        "//common/utils/file:file",
        "//common/proto:cc_map_proto",
        "//common/proto:cc_route_proto",
        "@glog",
    ],   
)