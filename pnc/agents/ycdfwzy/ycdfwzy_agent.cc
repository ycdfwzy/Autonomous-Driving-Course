#include "ycdfwzy_agent.h"
#include <algorithm>

namespace ycdfwzy
{

using std::cout;
using std::endl;
using std::to_string;
using interface::geometry::Point3D;
using interface::geometry::Point2D;
using interface::geometry::Vector3d;
using interface::agent::AgentStatus;
const double preview_length_max = 1.5;
const double preview_length_min = 0;
const double preview_length_th_max = 10;
const double preview_length_th_min = 5;
using std::min;
using std::max;
using std::abs;
using std::vector;
using std::string;
const int infd = (1<<30);
const double inff = 1e8;

int toInt(const string& st){
	int ret = 0;
	for (int i = 0; i < st.length(); ++i)
		if (st[i] >= '0' && st[i] <= '9')
			ret = ret*10+(st[i]-'0');
	return ret;
}
bool operator == (const interface::geometry::Point2D& p1, const interface::geometry::Point2D& p2){
	return std::fabs(p1.x()-p2.x()) < 1e-3 && std::fabs(p1.y()-p2.y()) < 1e-3;
}
inline double sqr(double x) {return x*x;}
inline double distance(interface::geometry::Point2D& p1,interface::geometry::Point2D& p2)
{
	return std::sqrt( sqr(p1.x()-p2.x()) + sqr(p1.y()-p2.y()) );
}
inline bool equal_to_zero(double x) { return (std::fabs(x)<1e-3); }
inline double get_velocity_len(const AgentStatus& agent_status){
	const auto& v = agent_status.vehicle_status().velocity();
	return std::sqrt(sqr(v.x())+sqr(v.y()));
}
inline double length(const Point2D& p) {return std::sqrt(sqr(p.x())+sqr(p.y()));}
inline double get_velocity_angle(const AgentStatus& agent_status){
	const auto& v = agent_status.vehicle_status().velocity();
	return atan2(v.y(), v.x());
}
inline void flat(const Point3D& s, Point2D& t){
	t.set_x(s.x()); t.set_y(s.y());
}
inline void flat(const Vector3d& s, Point2D& t){
	t.set_x(s.x()); t.set_y(s.y());
}
inline double mult(const Point2D& p1, const Point2D& p2){
	return p1.x()*p2.y()-p1.y()*p2.x();
}
Point2D operator -(const Point2D& p1, const Point2D& p2){
	Point2D ret;
	ret.set_x(p1.x()-p2.x());
	ret.set_y(p1.y()-p2.y());
	return ret;
}
Point2D operator +(const Point2D& p1, const Point2D& p2){
	Point2D ret;
	ret.set_x(p1.x()+p2.x());
	ret.set_y(p1.y()+p2.y());
	return ret;	
}
Point2D operator /(const Point2D& p, const int& a){
	Point2D ret;
	ret.set_x(p.x()/a);
	ret.set_y(p.y()/a);
	return ret;	
}
Point2D get_center(const Obstacle& obj){
	Point2D ret;
	ret.set_x(0); ret.set_y(0);
	for (const auto& point: obj.polygon_point()){
		Point2D tmp;
		flat(point, tmp);
		ret = ret + tmp;
	}
	ret = ret / obj.polygon_point_size();
	return ret;
}
double decide_velocity_for_obstacle(double dis, double ratio){
	dis = dis-1.5/ratio>0?dis-1.5/ratio:0;
	double minx = 2*ratio;
	double maxx = 8*ratio;
	if (dis < minx) return 0;
	if (dis > maxx) return max_velocity;
	return max_velocity/30*(dis-minx)/(maxx-minx);
	//return 0;
}


void SimpleVehicleAgent::Initialize(const interface::agent::AgentStatus& agent_status)
{
	string mapfilename = fileprefix+"pnc/agents/ycdfwzy/processed_map_proto.txt";
	CHECK(file::ReadFileToProto(mapfilename, &mapdata));
	angle_pid.set_pid(16, 0, 2);
	distance_pid.set_pid(-1, 0, 0);
	velocity_pid.set_pid(0.4, 0, 0);
	init_param(agent_status);
}

Command SimpleVehicleAgent::RunOneIteration(
	const interface::agent::AgentStatus& agent_status)
{
	Command command;
	 if ((finish_this_trip() && equal_to_zero(get_velocity_len(agent_status)))){
	 	// reached!
	 	busy = false;
	 }
	if ( agent_status.route_status().is_new_request() ){
		Initialize_Before_Start(agent_status);
		busy = true;
	}
	if (!busy)
		return command;
	command = get_command(agent_status);
	PublishVariable("nearest_point_id", to_string(nearest_id));
	return command;
}

Command SimpleVehicleAgent::get_command(const AgentStatus& agent_status){
	Command command;
	update_nearest_id(agent_status);
	// steer
    double steer_command = get_command_steer(agent_status);
    command.set_steering_angle(steer_command);
    command.set_steering_rate(0);

    // velocity
    double velocity_command = get_command_velocity(agent_status);
    if (velocity_command > 0)
    	command.set_throttle_ratio(velocity_command);
    else{
    	if (velocity_command < -1) velocity_command = -1;
    	command.set_brake_ratio(-velocity_command);
    }

    //std::cout << "Velocity Control command " << velocity_command << std::endl;
    //std::cout << "Steer Control command " << steer_command << std::endl;
	PublishVariable("Velocity Control command ",to_string(velocity_command));
	PublishVariable("Steer Control command",to_string(steer_command));

	return command;
}

double SimpleVehicleAgent::get_command_steer(const AgentStatus& agent_status){
	double k1 = get_command_distance(agent_status);
	double k2 = get_command_angle(agent_status);

	PublishVariable("Angle err",to_string(k2));
	PublishVariable("Distance err",to_string(k1));

	return k1 + k2;
}

double SimpleVehicleAgent::get_command_distance(const AgentStatus& agent_status){
	double err = get_distance_error(agent_status);
	//std::cout << "distance error: " << err << std::endl;

	distance_I += err;
	double distance_delta = 0;
	if (angle_last != 0)
		distance_delta = err - distance_last;
	distance_last = err;

	double ret = distance_pid.P * err + distance_pid.I * distance_I + distance_pid.D * distance_delta;
	return ret;
}

double SimpleVehicleAgent::get_command_angle(const AgentStatus& agent_status){
	double err = get_angle_error(agent_status);
	//std::cout << "angle error: " << err << std::endl;

	angle_I += err;
	double angle_delta = 0;
	if (angle_last != 0)
		angle_delta = err - angle_last;
	angle_last = err;

	double ret = angle_pid.P * err + angle_pid.I * angle_I + angle_pid.D * angle_delta;
	return ret;
}

double SimpleVehicleAgent::get_command_velocity(const AgentStatus& agent_status){
	double target = get_target_velocity(agent_status);
	double v = get_velocity_len(agent_status);

	velocity_I += target;
	double velocity_delta = 0;
	if (velocity_last != 0)
		velocity_delta = target - velocity_last;
	velocity_last = target;

	Point2D pos;
	flat(agent_status.vehicle_status().position(), pos);
	//pos.set_x(agent_status.vehicle_status().position().x());
	//pos.set_y(agent_status.vehicle_status().position().y());
	if (distance(pos, dest) < 20 && distance(pos, dest) > 2 && v < 0.5)
		return 0.2;
	if (distance(pos, dest) < 2 && v > 0.5)
		return -1;
	if (distance(pos, route[nearest_id]) < 2 && target < 1e-3 && v < 1)
		return -1;
	return velocity_pid.P * target + velocity_pid.I * velocity_I + velocity_pid.D * velocity_delta;
}

double SimpleVehicleAgent::get_distance_error(const AgentStatus& agent_status){
	Point2D p0;
	flat(agent_status.vehicle_status().position(), p0);
	//p0.set_x(agent_status.vehicle_status().position().x());
	//p0.set_y(agent_status.vehicle_status().position().y());
	Point2D p1 = route[nearest_id-1];
	Point2D p2 = route[nearest_id];

	Point2D t1 = p2-p1, t2 = p0-p1;
	double ret = mult(t1, t2)/length(t1);
	//std::cout << "posx: " << p0.x() << " posy: " << p0.y() << std::endl;
	//std::cout << "p1x: " << p1.x() << " p1y: " << p1.y() << std::endl;
	//std::cout << "s: " << mult(t1, t2) << " len: " << length(t1) << std::endl;
	return ret;
}

double SimpleVehicleAgent::get_angle_error(const AgentStatus& agent_status){
	double t1 = tan(get_velocity_angle(agent_status));

	Point2D p = get_future_pos(agent_status);
	int index = get_nearest_point_id(p, agent_status);

	if (index+1 == route.size())
		index--;
	Point2D q = route[index+1]-route[index];
	double t2 = q.y()/q.x();

	double ret = atan((t2-t1)/(1+t1*t2));
	return ret;
}

double SimpleVehicleAgent::get_target_velocity(const AgentStatus& agent_status){
	double ret = inff;
	ret = std::min(ret, calc_with_nothing(agent_status, nearest_id + 5, max_velocity));
	if (route.size() - nearest_id <= 5){
		ret = std::min(ret, calc_with_nothing(agent_status, route.size()-1, 0));
	}
	//std::cout << " " << ret << std::endl;
	consider_obstacle(agent_status, ret);
	consider_lights(agent_status, ret);
	//std::cout << " " << ret << std::endl;
	return ret;
}

double SimpleVehicleAgent::calc_with_nothing(const AgentStatus& agent_status, int to, double goal){
	Point2D pos;
	flat(agent_status.vehicle_status().position(), pos);
	double v = get_velocity_len(agent_status);
	double dis = distance(pos, route[nearest_id]);
	for (int i = nearest_id; i+1 <= to; ++i)
		dis += distance(route[i], route[i+1]);
	if (dis >= 4) dis -= 3.8;
	double ret = (sqr(goal)-sqr(v))/(2*dis);
	return ret;
}

void SimpleVehicleAgent::consider_obstacle(const AgentStatus& agent_status, double& goal){
	//Point2D pos;
	//flat(agent_status.vehicle_status().position(), pos);
	for (const Obstacle& obj : agent_status.perception_status().obstacle()){
		Point2D center = get_center(obj);
		int index = get_nearest_point_id(center, agent_status, 0);
		if (index == nearest_id || index > nearest_id+10)
			continue;
		if (index > nearest_id + 2) index -= 2;
		double small_goal = 0;
		if (obj.type() == interface::perception::ObjectType::CAR){
			double alpha = get_velocity_angle(agent_status);
			double beta = obj.heading();
			if (std::fabs((alpha) - (beta)) > 2.5)
				continue;
			// PublishVariable("delta alpha: ", to_string(std::fabs((alpha) - (beta))));
			// PublishVariable("alpha: ", to_string(alpha));
			// PublishVariable("beta: ", to_string(beta));
			small_goal = decide_velocity_for_obstacle(distance(center, route[index]), 1.0);
		}
		else
		if (obj.type() == interface::perception::ObjectType::PEDESTRIAN){
			//std::cout << distance(center, route[index]) << std::endl;
			//std::cout << index << " " << nearest_id << std::endl;
			small_goal = decide_velocity_for_obstacle(distance(center, route[index]), 0.5);
		}
		//std::cout << " " << goal << " " << small_goal << std::endl;
		goal = std::min(goal, calc_with_nothing(agent_status, index, small_goal));
		//std::cout << " " << goal << std::endl;
	}
}

void SimpleVehicleAgent::consider_lights(const AgentStatus& agent_status, double& goal){
	Point2D pos;
	flat(agent_status.vehicle_status().position(), pos);
	bool red = false;

	double dis = distance(le, pos);
	if (dis > p2le)
		update_lane(agent_status);
	else p2le = dis;
	for (const auto& lights: agent_status.perception_status().traffic_light()){
		for (const auto& light: lights.single_traffic_light_status()){
			std::string tmp = light.id().id();
			std::string id; id.clear();
			id += tmp[3];
			if (tmp[4] >= '0' & tmp[4] <= '9')
				id += tmp[4];
			int index = std::stoi(id);
			if (index == cur_laneId){
				if (light.color() != 2)
					red = true;
				break;
			}
		}
	}
	if (dis > 20 || !red) return;
	if (dis > 3.8) dis -= 3.8;
		else dis = 1e-3;
	goal = std::min(goal, -sqr(get_velocity_len(agent_status))/2/dis);
}

void SimpleVehicleAgent::Initialize_Before_Start(const AgentStatus& agent_status){
	get_route(agent_status);
	init_param(agent_status);
}

void SimpleVehicleAgent::get_route(const AgentStatus& agent_status){
	Point3D sta, des = agent_status.route_status().destination();
	sta.set_x(agent_status.vehicle_status().position().x());
	sta.set_y(agent_status.vehicle_status().position().y());
	sta.set_z(agent_status.vehicle_status().position().z());

	flat(sta, start);
	flat(des, dest);

	route.clear();
	target_velocity.clear();
	interface::route::Route tmp_route;
	FindRoute(mapdata, sta, des, tmp_route);

	int N = tmp_route.route_point_size();
	for (int i = 0; i < N; ++i){
		route.push_back(tmp_route.route_point(i));
		//std::cout << "x: " << route.back().x() << " y: " << route.back().y() << std::endl;
		target_velocity.push_back(0);
	}
}

Point2D SimpleVehicleAgent::get_future_pos(const AgentStatus& agent_status){
	Point2D ret;
	const auto& pos = agent_status.vehicle_status().position();
	double v = get_velocity_len(agent_status);
	double alpha = get_velocity_angle(agent_status);
	double len = (v < preview_length_th_min) ? preview_length_min :
				((v > preview_length_th_max) ? preview_length_max :
				 (v-preview_length_th_min)*(preview_length_max-preview_length_min)/
				 (preview_length_th_max-preview_length_th_min)+preview_length_min);
	ret.set_x(pos.x()+len*cos(alpha));
	ret.set_y(pos.y()+len*sin(alpha));
	return ret;
}

void SimpleVehicleAgent::update_nearest_id(const AgentStatus& agent_status){
	Point3D tmp;
	tmp.set_x(agent_status.vehicle_status().position().x());
	tmp.set_y(agent_status.vehicle_status().position().y());
	Point2D cur_pos;
	flat(tmp, cur_pos);
	while (nearest_id+1 < route.size() && distance(cur_pos, route[nearest_id]) > distance(cur_pos, route[nearest_id+1]) )
		++nearest_id;
}

int SimpleVehicleAgent::get_nearest_point_id(Point2D p, const AgentStatus&, int threshold){
	int ret = 0;
	int N = route.size();
	double minx = inff;
	int t = 0;
	for (int i = nearest_id; i < N; ++i){
		double d = distance(route[i], p);
		if (d < minx){
			minx = d;
			ret = i;
		} else
			t++;
		if (t > threshold) break;
	}
	return ret;
}

void SimpleVehicleAgent::init_param(const AgentStatus& agent_status){
	nearest_id = 1;
	angle_I = 0;
	distance_I = 0;
	velocity_I = 0;
	angle_last = 0;
	distance_last = 0;
	velocity_last = 0;
	update_lane(agent_status);
}

void SimpleVehicleAgent::update_lane(const AgentStatus& agent_status){
	Point2D pos;
	flat(agent_status.vehicle_status().position(), pos);

	int laneId = FindLaneId(pos, mapdata);
	if (laneId == cur_laneId) return;
	cur_laneId = laneId;
	interface::map::Lane lane = FindCorrectLane(laneId, mapdata);
	const auto& central = lane.central_line();

	int N = central.point_size();
	flat(central.point(N-1), le);
	p2le = distance(le, pos);
}

int FindLaneId(const interface::geometry::Point2D& testpoint, const interface::map::Map& mapdata)
{
	for(const auto& lane : mapdata.lane())
	{
		interface::geometry::Point3D l0,l1,r0,r1;
		int lsize=lane.left_bound().boundary().point_size();
		int rsize=lane.right_bound().boundary().point_size();
		l0=lane.left_bound().boundary().point(0);
		l1=lane.left_bound().boundary().point(lsize-1);
		r0=lane.right_bound().boundary().point(0);
		r1=lane.right_bound().boundary().point(rsize-1);

		//use simple square box to decide
		double xmin=min(min(l0.x(),l1.x()),min(r0.x(),r1.x()));
		double ymin=min(min(l0.y(),l1.y()),min(r0.y(),r1.y()));
		double xmax=max(max(l0.x(),l1.x()),max(r0.x(),r1.x()));
		double ymax=max(max(l0.y(),l1.y()),max(r0.y(),r1.y()));
		if(testpoint.x()>=xmin && testpoint.x()<=xmax && testpoint.y()>=ymin && testpoint.y()<=ymax)
			return toInt(lane.id().id());
	}
	return -1;
}

interface::map::Lane FindCorrectLane(int id,const interface::map::Map& map) {
	for(const auto& lane : map.lane())
		if ( toInt(lane.id().id()) == id )
			return lane;
}

int find_closest_point(interface::geometry::Point2D& p,
				  	   const interface::map::Lane& lane) {
	double mind = inff;
	int N = lane.central_line().point_size();
	for (int i = 0; i < N; ++i) {
		interface::geometry::Point2D point;
		point.set_x(lane.central_line().point(i).x());
		point.set_y(lane.central_line().point(i).y());
		double d = distance(p, point);
		if (d > mind) return i-1;
		mind = d;
	}
	return 0;
}

//dfs for outputing path
void dfs(int x, int* path, std::pair<int, int> sta, interface::route::Route& route, const interface::map::Map& map_data){
	if (x == sta.first){
		int m = map_data.lane(x).central_line().point().size();
		for (int i = sta.second; i < m; ++i){
			interface::geometry::Point2D tmp;
			tmp.set_x(map_data.lane(x).central_line().point(i).x());
			tmp.set_y(map_data.lane(x).central_line().point(i).y());
			if (!(route.route_point(route.route_point_size()-1) == tmp)){
				//std::cout << tmp.x() << " " << tmp.y() << std::endl;
				interface::geometry::Point2D* p = route.add_route_point();
				p->CopyFrom(tmp);
			}
			// interface::geometry::Point2D* p = route.add_route_point();
			// p->set_x(map_data.lane(x).central_line().point(i).x());
			// p->set_y(map_data.lane(x).central_line().point(i).y());
		}
		//std::cout << std::endl;
		return;
	}

	dfs(path[x], path, sta, route, map_data);
	int m = map_data.lane(x).central_line().point_size();
	for (int i = 0; i < m; ++i){
		interface::geometry::Point2D tmp;
		tmp.set_x(map_data.lane(x).central_line().point(i).x());
		tmp.set_y(map_data.lane(x).central_line().point(i).y());
		if (!(route.route_point(route.route_point_size()-1) == tmp)){
			interface::geometry::Point2D* p = route.add_route_point();
			p->CopyFrom(tmp);
		}
		// interface::geometry::Point2D* p = route.add_route_point();
		// p->set_x(map_data.lane(x).central_line().point(i).x());
		// p->set_y(map_data.lane(x).central_line().point(i).y());
	}
}

void normal(interface::route::Route& route){
	std::vector<interface::geometry::Point2D> rt;
	rt.clear();
	for (const auto& point: route.route_point()){
		//std::cout << point.x() << " " << point.y() << std::endl;
		rt.push_back(point);
	}
	route.clear_route_point();
	int tot_point = rt.size();
	interface::geometry::Point2D last = rt[0];
	
	auto* tmp = route.add_route_point();
	tmp->CopyFrom(last);

	for (int i = 1; i < tot_point; ++i){
		if (i+1 < tot_point && distance(last, rt[i]) < 2 ||
			i+1 == tot_point && distance(last, rt[i]) < 1)
			continue;

		last.CopyFrom(rt[i]);
		tmp = route.add_route_point();
		tmp->CopyFrom(last);
	}
	//for (const auto& point: route.route_point()){
	//	std::cout << point.x() << " " << point.y() << std::endl;
		//rt.push_back(point);
	//}
}

void get_route(int start_id, int s, int end_id, int e, int* map_id_index, int lane_size,
			   const interface::map::Map& mapdata,
			   interface::geometry::Point2D end_point,
			   interface::route::Route& route){
	int* path = new int[lane_size+1];
	int* q = new int[lane_size+1];
	bool* flag = new bool[lane_size+1];
	memset(flag, false, sizeof(bool)*(lane_size+1));
	q[1] = start_id;
	//BFS search on lanes
	int l, r;
	for (l=1, r=1; l<=r; ++l){
		int i = q[l];
		const auto& lane = mapdata.lane(i);

		for (const auto& la: lane.successor()){
			int j = map_id_index[toInt(la.id())];
			if (!flag[j]){
				flag[j] = true;
				path[j] = i;
				q[++r] = j;
			}
		}
	}

	if (!flag[end_id]){
		std::cout << "No Route" << std::endl;
	} else
	{
		dfs(path[end_id], path, std::make_pair(start_id, s), route, mapdata);

		//int e = find_closest_point(end_point, lane);
		for (int i = 0; i <= e; ++i){
			interface::geometry::Point2D tmp;
			tmp.set_x(mapdata.lane(end_id).central_line().point(i).x());
			tmp.set_y(mapdata.lane(end_id).central_line().point(i).y());
			if (!(route.route_point(route.route_point_size()-1) == tmp)){
				interface::geometry::Point2D* p = route.add_route_point();
				p->CopyFrom(tmp);
			}
			// auto* p = route.add_route_point();
			// p->set_x(mapdata.lane(end_id).central_line().point(i).x());
			// p->set_y(mapdata.lane(end_id).central_line().point(i).y());
		}
		if (!(route.route_point(route.route_point_size()-1) == end_point)){
			auto* p = route.add_route_point();
			p->CopyFrom(end_point);
		}
	}

	delete[] map_id_index;
	delete[] path;
	delete[] q;
	delete[] flag;

	normal(route);
}

//inputfilename is the proto file containing starting and ending point
void FindRoute(const interface::map::Map& mapdata,
			   const interface::geometry::Point3D& start3d,
			   const interface::geometry::Point3D& ene3d,
			   interface::route::Route& route) {
	interface::geometry::Point2D start_point, end_point;
	start_point.set_x(start3d.x());
	start_point.set_y(start3d.y());
	end_point.set_x(ene3d.x());
	end_point.set_y(ene3d.y());
	
	int start_id = FindLaneId(start_point, mapdata);
	int end_id = FindLaneId(end_point, mapdata);

	//if (start_id==-1 || end_id==-1) {
	//	std::cout << "Point could not been find on the map" << std::endl;
	//	return;
	//}
	int lane_size = mapdata.lane_size();
	int* map_id_index = new int[lane_size+1];
	for (int i = 0; i < lane_size; ++i){
		int t = toInt(mapdata.lane(i).id().id());
		map_id_index[t] = i;
	}
	//std::cout << start_point.x() << " " << start_point.y() << std::endl;
	start_id = map_id_index[start_id];
	end_id = map_id_index[end_id];
	//std::cout << route.route_point_size() << std::endl;
	auto* p = route.add_route_point();
	p->CopyFrom(start_point);
	//std::cout << route.route_point_size() << std::endl;
	//std::cout << start_point.x() << " " << start_point.y() << std::endl;

	interface::map::Lane slane = mapdata.lane(start_id);//FindCorrectLane(start_id, lane);
	interface::map::Lane elane = mapdata.lane(end_id);
	int s = find_closest_point(start_point, slane);
	int e = find_closest_point(end_point, elane);

	//std::cout << start_point.x() << " " << start_point.y() << std::endl;
	//std::cout << slane.central_line().point(s).x() << " " << slane.central_line().point(s).y() << std::endl;
	if (start_id == end_id && s <= e){
		//auto* p = route.add_route_point();
		//p->CopyFrom(start_point);
		for (int i = s;i <= e; ++i){
			interface::geometry::Point2D tmp;
			tmp.set_x(slane.central_line().point(i).x());
			tmp.set_y(slane.central_line().point(i).y());
			if (!(route.route_point(route.route_point_size()-1) == tmp)){
				p = route.add_route_point();
				p->set_x(slane.central_line().point(i).x());
				p->set_y(slane.central_line().point(i).y());
			}
		}
		p = route.add_route_point();
		p->CopyFrom(end_point);
		normal(route);
		return;
	}
	if (start_id == end_id && s > e){
		int N = slane.central_line().point_size();
		for (int i = s; i < N; ++i){
			interface::geometry::Point2D tmp;
			tmp.set_x(slane.central_line().point(i).x());
			tmp.set_y(slane.central_line().point(i).y());
			if (!(route.route_point(route.route_point_size()-1) == tmp)){
				p = route.add_route_point();
				p->set_x(slane.central_line().point(i).x());
				p->set_y(slane.central_line().point(i).y());
			}
			// p = route.add_route_point();
			// p->set_x(slane.central_line().point(i).x());
			// p->set_y(slane.central_line().point(i).y());
		}
		start_id = map_id_index[toInt(slane.successor(0).id())];
		s = 0;
		get_route(start_id, s, end_id, e, map_id_index, lane_size, mapdata, end_point, route);
		return;
	}
	get_route(start_id, s, end_id, e, map_id_index, lane_size, mapdata, end_point, route);
}

}