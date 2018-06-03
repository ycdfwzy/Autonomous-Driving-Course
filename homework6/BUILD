package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "simulation_main",
    srcs = ["simulation_main.cc"],
    deps = [
        "//homework6/display:simulation_main_window",
        "@gflags//:gflags",
        "@glog",
    ],
)

cc_library(
	name = "generatetable",
	srcs = ["GenerateTable.cc"],
	hdrs = ["GenerateTable.h"],
	deps = [
		"//homework6/simulation:vehicle_agent",
		"//common/proto:cc_agent_status_proto",
        "//common/proto:cc_map_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//common/proto:cc_vehicle_params_proto",
        "//homework6/simulation/dynamic_lib:libdynamic",
        "//homework6/proto:cc_simulation_config_proto",
        "//common/utils/file",
        "//common/utils/file:path",
        "@gflags//:gflags",
        "@glog",
	],
)

cc_binary(
	name = "generatetable_main",
	srcs = ["GenerateTable_main.cc"],
	deps = [
		":generatetable"
	],
)