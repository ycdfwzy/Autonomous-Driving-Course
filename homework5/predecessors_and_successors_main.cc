#include "homework5/predecessors_and_successors.h"
#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"
#include "glog/logging.h"
#include <iostream>

using homework5::map::MapLib;
using namespace std;

int main(){
	MapLib ml;
	// ATTENTION!!! : please use absolute path for reading the data file.
	FindPreSuc(ml.map_proto(), "/home/ycdfwzy/autonomous_driving/PublicCourse/homework5/map/grid2/processed_map_proto.txt");
	return 0;
}