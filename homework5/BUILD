package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_visualizer_main",
    srcs = ["map_visualizer_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework5/display:main_window",
        "//homework5/map:map_lib",
        "@gflags//:gflags",
        "@glog",
    ],
)

cc_library(
	name = "predecessors_and_successors",
	srcs = ["predecessors_and_successors.cc"],
	hdrs = [
		"predecessors_and_successors.h",
	],
	deps = [
		"//homework5/map:map_lib",
		"//common/utils/file:file",
        "//common/proto:cc_map_proto",
        "@glog",
	],

)

cc_binary(
	name = "predecessors_and_successors_main",
	srcs = ["predecessors_and_successors_main.cc"],
	deps = [
		"//homework5:predecessors_and_successors",
		"//common/utils/file:file",
        "//common/proto:cc_map_proto",
        "@glog",
	],
)

cc_library(
	name = "route",
	srcs = ["route.cc"],
	hdrs = ["route.h"],
	deps = [
		"//homework5:predecessors_and_successors",
		"//homework5/map:map_lib",
		"//common/utils/file:file",
        "//common/proto:cc_map_proto",
        "//common/proto:cc_route_proto",
        "@glog",
	],
)

cc_binary(
	name = "route_main",
	srcs = ["route_main.cc"],
	deps = [
		"//homework5:route",
		"//homework5:predecessors_and_successors",
		"//homework5/map:map_lib",
		"//common/utils/file:file",
        "//common/proto:cc_map_proto",
        "//common/proto:cc_route_proto",
        "@glog",
	],
)
