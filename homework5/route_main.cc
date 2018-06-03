#include "homework5/route.h"
#include "homework5/predecessors_and_successors.h"
#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"
#include "common/proto/route.pb.h"
#include "glog/logging.h"
#include <iostream>
#include <string>

using namespace std;

void mainwork(const string& base_path, const string& infilename, const string& outfilename){
	interface::map::Map map;
	CHECK(file::ReadFileToProto("homework5/map/grid2/processed_map_proto.txt", &map));
	interface::route::Route rt;
	CHECK(file::ReadFileToProto(base_path+infilename, &rt));
	find_route(rt, map, base_path+outfilename);
}

int main(){
	// ATTENTION!!! : please use absolute path for reading the data file.
	mainwork("/home/ycdfwzy/autonomous_driving/PublicCourse/homework5/data/routes/",
		"route_request_1.txt", "route_result_1.txt");
	mainwork("/home/ycdfwzy/autonomous_driving/PublicCourse/homework5/data/routes/",
		"route_request_2.txt", "route_result_2.txt");
	mainwork("/home/ycdfwzy/autonomous_driving/PublicCourse/homework5/data/routes/",
		"route_request_3.txt", "route_result_3.txt");
	mainwork("/home/ycdfwzy/autonomous_driving/PublicCourse/homework5/data/routes/",
		"route_request_4.txt", "route_result_4.txt");
	mainwork("/home/ycdfwzy/autonomous_driving/PublicCourse/homework5/data/routes/",
		"route_request_5.txt", "route_result_5.txt");
	return 0;
}