#pragma once

#ifndef ycdfwzy_agent_
#define ycdfwzy_agent_

#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pnc/simulation/vehicle_agent.h"
#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "common/utils/file/file.h"
#include "common/proto/vehicle_params.pb.h"
#include "pnc/simulation/vehicle_agent_factory.h"
#include "common/proto/route.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/transform.pb.h"
#include "glog/logging.h"

using std::pair;
using std::string;

namespace ycdfwzy
{

const double max_velocity = 13;

struct PID
{
	double P, I, D;
	PID(double P_=0, double I_=0, double D_=0){
		P = P_;
		I = I_;
		D = D_;
	}
	void set_pid(double P_, double I_, double D_){
		P = P_;
		I = I_;
		D = D_;
	}
};

typedef interface::control::ControlCommand Command;
typedef interface::perception::PerceptionObstacle Obstacle;
typedef interface::perception::SingleTrafficLightStatus TrafficLight;
typedef interface::perception::PerceptionTrafficLightStatus AllTrafficLight;

int FindLaneId(const interface::geometry::Point2D& testpoint, const interface::map::Map& mapdata);
interface::map::Lane FindCorrectLane(int id,const interface::map::Map& map);
int find_closest_point(interface::geometry::Point2D& p,
				  	   const interface::map::Lane& lane);
void dfs(int x, int* path, std::pair<int, int> sta, interface::route::Route& route, const interface::map::Map& map_data);
void normal(interface::route::Route& route);
void get_route(int start_id, int s, int end_id, int e, int* map_id_index, int lane_size,
			   const interface::map::Map& mapdata,
			   interface::geometry::Point2D end_point,
			   interface::route::Route& route);
void FindRoute(const interface::map::Map& mapdata,
			   const interface::geometry::Point3D& start3d,
			   const interface::geometry::Point3D& ene3d,
			   interface::route::Route& route);

class SimpleVehicleAgent : public simulation::VehicleAgent {
public:
	explicit SimpleVehicleAgent(const std::string& name) : VehicleAgent(name) {}

	virtual void Initialize(const interface::agent::AgentStatus& agent_status);

	virtual interface::control::ControlCommand RunOneIteration(
	    const interface::agent::AgentStatus& agent_status);

	Command get_command(const interface::agent::AgentStatus&);
	void update_nearest_id(const interface::agent::AgentStatus&);
	void Initialize_Before_Start(const interface::agent::AgentStatus&);
	double get_command_steer(const interface::agent::AgentStatus&);
	double get_command_velocity(const interface::agent::AgentStatus&);
	double get_command_angle(const interface::agent::AgentStatus&);
	double get_command_distance(const interface::agent::AgentStatus&);
	double get_target_velocity(const interface::agent::AgentStatus&);
	double get_distance_error(const interface::agent::AgentStatus&);
	double get_angle_error(const interface::agent::AgentStatus&);
	void get_route(const interface::agent::AgentStatus&);
	interface::geometry::Point2D get_future_pos(const interface::agent::AgentStatus&);
	int get_nearest_point_id(interface::geometry::Point2D, const interface::agent::AgentStatus&, int threshold=5);
	void init_param(const interface::agent::AgentStatus&);
	void update_lane(const interface::agent::AgentStatus&);
	double calc_with_nothing(const interface::agent::AgentStatus&, int, double);
	void consider_obstacle(const interface::agent::AgentStatus&, double&);
	void consider_lights(const interface::agent::AgentStatus&, double&);
	bool finish_this_trip(){
		return (nearest_id+1 == route.size());
	}

private:
	//PLEASE CHANGE THE FILEPREFIX TO YOUR WORKING PATH BEFORE YOU RUN THIS CODE
	string fileprefix="/home/ycdfwzy/autonomous_driving/PublicCourse/";

	PID angle_pid, distance_pid, velocity_pid;

	std::vector< interface::geometry::Point2D > route;
	std::vector< double > target_velocity;
	/*--------some parameters for status--------*/
	bool busy;	//wheather in some trip
	interface::geometry::Point2D start, dest;
	int nearest_id; // nearest point id
	int cur_laneId;
	interface::geometry::Point2D le;
	double angle_I;
	double distance_I;
	double velocity_I;
	double angle_last;
	double distance_last;
	double velocity_last;
	double p2le;
	/*-----------------------------------------*/

	interface::map::Map mapdata;
};

}

#endif