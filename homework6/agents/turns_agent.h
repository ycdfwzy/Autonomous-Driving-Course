#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"
#include "homework5/route.h"
#include "homework5/predecessors_and_successors.h"
#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"
#include "common/proto/route.pb.h"
#include "glog/logging.h"
#include "homework6/map/map_lib.h"
#include <iostream>
#include <cmath>
#include <string>

namespace ycdfwzy {

class TurnsAgent : public simulation::VehicleAgent{

public:
	explicit TurnsAgent(const std::string& name) : VehicleAgent(name) {}

	void Initialize(const interface::agent::AgentStatus&  agent_status);
	interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status);


private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    ;
    return std::sqrt(sqr_sum);
  }
  double CalcDistance2D(const interface::geometry::Point2D& p1,
                      const interface::geometry::Point2D& p2) {
    double sqr_sum =
        math::Sqr(p1.x() - p2.x()) + math::Sqr(p1.y() - p2.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcAcceleration(const interface::geometry::Vector3d& acceleration){
  	double sqr_sum = math::Sqr(acceleration.x()) + math::Sqr(acceleration.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double mult(const interface::geometry::Point2D& p1,
              const interface::geometry::Point2D& p2){
  	return p1.x()*p2.y()-p2.x()*p1.y();
  }

  int turning_angle(int index){
  	/*
  	if (now_index == 0 || now_index+1 > rt.route_point().size()) return false;
  	interface::geometry::Point2D p1, p2;
  	p1.set_x(rt.route_point()[now_index].x() - rt.route_point()[now_index-1].x());
  	p1.set_y(rt.route_point()[now_index].y() - rt.route_point()[now_index-1].y());
  	p2.set_x(rt.route_point()[now_index+1].x() - rt.route_point()[now_index-1].x());
  	p2.set_y(rt.route_point()[now_index+1].y() - rt.route_point()[now_index-1].y());
  	return mult(p1, p2);*/
  	const auto& lane = map.lane()[index];
  	int n = lane.central_line().point().size();
  	const auto& s = lane.central_line().point()[0];
  	const auto& m = lane.central_line().point()[n/2];
  	const auto& e = lane.central_line().point()[n-1];
  	interface::geometry::Point2D p1, p2;
  	p1.set_x(m.x() - s.x());
  	p1.set_y(m.y() - s.y());
  	p2.set_x(e.x() - s.x());
  	p2.set_y(e.y() - s.y());
  	if (fabs(p1.x()-p2.x()) < 0.1 || fabs(p1.y()-p2.y()) < 0.1)
  		return 0;

  	std::cout << p1.x() << " " << p1.y() << std::endl;
  	std::cout << p2.x() << " " << p2.y() << std::endl << std::endl;
  	double d = mult(p1, p2);
  	if (d > 1e-2) return 1;
  	if (d + 1e-2 < 0) return -1;
  	return 0;
  }
  std::pair<int, int> find_closest_point(interface::geometry::Point2D& p, const interface::map::Map& map_data);

  bool position_reached_destination_ = false;
  bool velocity_reached_threshold_ = false;
  double last_velocity;
  interface::route::Route rt;
  interface::map::Map map;
  int now_index;
  int last_isturing;
  int count;
};

} // namespace ycdfwzy