#include "homework5/predecessors_and_successors.h"
#include <cmath>
#include <iostream>
//compare Point3D
bool operator ==(const interface::geometry::Point3D& p0, const interface::geometry::Point3D& p1){
	#define eps 1e-8
	return std::fabs(p0.x()-p1.x()) < eps && std::fabs(p0.y()-p1.y()) < eps && std::fabs(p0.z()-p1.z()) < eps;
}

// find predecessor and successor
void FindPreSuc(const interface::map::Map& map_data_, const std::string path){
	interface::map::Map PreSuc;
	int n = map_data_.lane().size();
	std::cout << n << std::endl;
	for (int i = 0; i < n; ++i){
		interface::map::Lane lane;
		lane.CopyFrom(map_data_.lane()[i]);
		int lbtot = lane.left_bound().boundary().point().size();
		int rbtot = lane.right_bound().boundary().point().size();
		//start two points and end two points of a lane
		const interface::geometry::Point3D& lb0 = lane.left_bound().boundary().point()[0];
		const interface::geometry::Point3D& lb1 = lane.left_bound().boundary().point()[lbtot-1];
		const interface::geometry::Point3D& rb0 = lane.right_bound().boundary().point()[0];
		const interface::geometry::Point3D& rb1 = lane.right_bound().boundary().point()[rbtot-1];
		for (int j = 0; j < n; ++j){
			interface::map::Lane lane2;
			lane2.CopyFrom(map_data_.lane()[j]);
			int lbtot2 = lane2.left_bound().boundary().point().size();
			int rbtot2 = lane2.right_bound().boundary().point().size();
			//start two points and end two points of a lane
			const interface::geometry::Point3D& lb02 = lane2.left_bound().boundary().point()[0];
			const interface::geometry::Point3D& lb12 = lane2.left_bound().boundary().point()[lbtot2-1];
			const interface::geometry::Point3D& rb02 = lane2.right_bound().boundary().point()[0];
			const interface::geometry::Point3D& rb12 = lane2.right_bound().boundary().point()[rbtot2-1];

			//predecessor: lane2's end is lane's start
			if (lb0 == lb12 && rb0 == rb12){
				interface::map::Id* ID = lane.add_predecessor();
				ID->set_id(lane2.id().id());
			} else
			//successor: lane's end is lane2's start
			if (lb1 == lb02 && rb1 == rb02){
				interface::map::Id* ID = lane.add_successor();
				ID->set_id(lane2.id().id());
			}
		}
		interface::map::Lane* l = PreSuc.add_lane();
		l->CopyFrom(lane);
	}
	//write to file
	CHECK(file::WriteProtoToTextFile(PreSuc, path));
}