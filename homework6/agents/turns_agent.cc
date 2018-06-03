#include "homework6/agents/turns_agent.h"

namespace ycdfwzy{

using namespace std;

pair<int, int> TurnsAgent::find_closest_point(interface::geometry::Point2D& p, const interface::map::Map& map_data){
	int n = map_data.lane().size();
	double t1 = -1, t2 = -1;
	double mind = 1e9;
	for (int i = 0; i < n; ++i){
		const auto& lane = map_data.lane()[i];
		int m = lane.central_line().point().size();
		for (int j = 0; j < m-1; ++j){
			const interface::geometry::Point3D& q = lane.central_line().point()[j];
			interface::geometry::Point2D qq;
			qq.set_x(q.x());
			qq.set_y(q.y());
			double d = CalcDistance2D(p, qq);
			if (d < mind){
				mind = d;
				t1 = i;
				t2 = j;
			}
		}
	}
	if (mind > 4){
		t1 = -1;
		t2 = -1;
	}
	return make_pair(t1, t2);
}

void TurnsAgent::Initialize(const interface::agent::AgentStatus& agent_status){
	rt.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
	rt.mutable_start_point()->set_y(agent_status.vehicle_status().position().y());
	rt.mutable_end_point()->set_x(agent_status.route_status().destination().x());
	rt.mutable_end_point()->set_y(agent_status.route_status().destination().y());

	CHECK(file::ReadFileToProto("homework5/map/grid2/processed_map_proto.txt", &map));
	find_route(rt, map, "route.txt");
	now_index = 0;
	count = 0;
	last_isturing = 0;
}

interface::control::ControlCommand TurnsAgent::RunOneIteration(
      const interface::agent::AgentStatus& agent_status){
	interface::control::ControlCommand command;


	interface::geometry::Point2D p1, p2;
	/*
	p1.set_x(agent_status.vehicle_status().position().x());
	p1.set_y(agent_status.vehicle_status().position().y());
	if (now_index+1 < rt.route_point().size() &&
		CalcDistance2D(p1, rt.route_point()[now_index]) > CalcDistance2D(p1, rt.route_point()[now_index+1]) ){
		now_index++;
	}
	*/
	//cout << now_index << endl;

	p1.set_x(agent_status.vehicle_status().position().x());
	p1.set_y(agent_status.vehicle_status().position().y());
	pair<int, int> pos = find_closest_point(p1, map);
	if (pos.first == -1){
		cout << "-1" << endl;
		return command;
	}
	int isturing = turning_angle(pos.first);
	cout << pos.first << endl;

	/*
	if (count > 0){
		count--;
		command.set_steering_angle(-last_isturing*2.456);
		if (count == 0) last_isturing = 0;
	}
	else if (isturing == 0 && last_isturing != 0){
		command.set_steering_angle(-last_isturing*2.456);
		count = 77;
	} else{
		command.set_steering_angle(isturing*2.456);
		last_isturing = isturing;
	}
	*/
	if (isturing == 0 && agent_status.vehicle_status().angular_velocity_vcs().z() > 0.01){
		command.set_steering_angle(-3.14);
	}
	if ((isturing == 0 && agent_status.vehicle_status().angular_velocity_vcs().z() < -0.01)){
		command.set_steering_angle(3.14);
	}
	else if (isturing == -1 || (isturing == 0 && agent_status.vehicle_status().angular_velocity_vcs().z() > 0.01)){
		command.set_steering_angle(-2.456);
	} else
	if (isturing == 1 || (isturing == 0 && agent_status.vehicle_status().angular_velocity_vcs().z() < -0.01)){
		command.set_steering_angle(2.456);
	}
/*
	double d = turning_angle();
	if (d > (0.01)){
		command.set_steering_angle(2.356);
	} else
	if (d + (0.01) < 0){
		command.set_steering_angle(-2.356);
	}
*/
	// Vehicle's current position reaches the destination
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 4.0) {
      position_reached_destination_ = true;
  	  command.set_brake_ratio(1.0);
  	  return command;
    }
    // Vehicle's current velocity reaches 5 m/s
    if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5) {
      velocity_reached_threshold_ = true;

      double e = (5.0 - CalcVelocity(agent_status.vehicle_status().velocity()));
      double d = CalcVelocity(agent_status.vehicle_status().velocity()) - last_velocity;
      double Kp = 5.1;
      double Kd = 5.1;
      double pd = Kp*e + Kd*d;
      if (pd > 1) pd = 1;
      if (pd < -1) pd = -1;
      if (pd > 0)
      	command.set_throttle_ratio(pd);
      else command.set_brake_ratio(-pd);

      return command;
    }
    command.set_throttle_ratio(0.3);
	last_velocity = CalcVelocity(agent_status.vehicle_status().velocity());
    return command;
}

} // namespace ycdfwzy