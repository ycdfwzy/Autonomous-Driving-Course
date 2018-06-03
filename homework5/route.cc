#include "homework5/route.h"
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;

inline double distance(interface::geometry::Point2D& p, interface::geometry::Point2D& q){
	return sqrt( (p.x()-q.x())*(p.x()-q.x())+(p.y()-q.y())*(p.y()-q.y()) );
}

//find proper point on lane
pair<int, int> find_closest_point(interface::geometry::Point2D& p, const interface::map::Map& map_data){
	int n = map_data.lane().size();
	double t1 = -1, t2 = -1;
	double mind = 1e9;
	for (int i = 0; i < n; ++i){
		const auto& lane = map_data.lane()[i];
		int m = lane.central_line().point().size();
		for (int j = 0; j < m; ++j){
			const interface::geometry::Point3D& q = lane.central_line().point()[j];
			interface::geometry::Point2D qq;
			qq.set_x(q.x());
			qq.set_y(q.y());
			double d = distance(p, qq);
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

//dfs for outputing path
void dfs(int x, int* path, pair<int, int> sta, interface::route::Route& rt, const interface::map::Map& map_data){
	if (x == sta.first){
		int m = map_data.lane()[x].central_line().point().size();
		for (int i = sta.second; i < m; ++i){
			interface::geometry::Point2D* p = rt.add_route_point();
			p->set_x(map_data.lane()[x].central_line().point()[i].x());
			p->set_y(map_data.lane()[x].central_line().point()[i].y());
		}
		return;
	}

	dfs(path[x], path, sta, rt, map_data);
/*
	if (x == des.first){
		//int m = map_data.lane()[x].central_line().point().size();
		for (int i = 0; i <= des.second; ++i){
			interface::geometry::Point2D* p = rt.add_route_point();
			p->set_x(map_data.lane()[x].central_line().point()[i].x());
			p->set_y(map_data.lane()[x].central_line().point()[i].y());
		}
	} else*/
	{
		int m = map_data.lane()[x].central_line().point().size();
		for (int i = 0; i < m; ++i){
			interface::geometry::Point2D* p = rt.add_route_point();
			p->set_x(map_data.lane()[x].central_line().point()[i].x());
			p->set_y(map_data.lane()[x].central_line().point()[i].y());
		}
	}
}

int toInt(const string& st){
	int ret = 0;
	for (int i = 0; i < st.length(); ++i)
		if (st[i] >= '0' && st[i] <= '9')
			ret = ret*10+(st[i]-'0');
	return ret;
}

void find_route(interface::route::Route& rt, const interface::map::Map& map_data, const std::string& filepath){
	interface::geometry::Point2D start = rt.start_point();
	interface::geometry::Point2D end = rt.end_point();
	pair<int, int> S = find_closest_point(start, map_data);
	pair<int, int> T = find_closest_point(end, map_data);
	if (S.first == -1){
		cout << "start not found!" << endl;
		return;
	}
	if (T.first == -1){
		cout << "end not found!" << endl;
		return;
	}
	// start and end points are on one lane
	if (S.first == T.first && S.second < T.second){
		int x = S.first;
		for (int i = S.second; i <= T.second; ++i){
			interface::geometry::Point2D* p = rt.add_route_point();
			p->set_x(map_data.lane()[x].central_line().point()[i].x());
			p->set_y(map_data.lane()[x].central_line().point()[i].y());
		}
		return;
	}

	int n = map_data.lane().size();
	int* path = new int[n+1];
	int* q = new int[n+1];
	bool* flag = new bool[n+1];
	memset(flag, false, sizeof(bool)*(n+1));
	q[1]=S.first;

	int* map_id_index = new int[n+1];
	for (int i = 0; i < n; ++i){
		int t = toInt(map_data.lane()[i].id().id());
		map_id_index[t] = i;
	}

	//BFS search on lanes
	int l, r;
	for (l=1, r=1; l<=r; ++l){
		int i = q[l];
		const auto& lane = map_data.lane()[i];

		for (const auto& la: lane.successor()){
			int j = map_id_index[toInt(la.id())];
			if (!flag[j]){
				flag[j] = true;
				path[j] = i;
				q[++r] = j;
			}
		}
	}

	if (!flag[T.first]){
		cout << "No Route" << endl;
	} else
	{
		dfs(path[T.first], path, S, rt, map_data);

		for (int i = 0; i < T.second; ++i){
			interface::geometry::Point2D* p = rt.add_route_point();
			p->set_x(map_data.lane()[T.first].central_line().point()[i].x());
			p->set_y(map_data.lane()[T.first].central_line().point()[i].y());
		}
	}

	delete[] map_id_index;
	delete[] path;
	delete[] q;
	delete[] flag;

	CHECK(file::WriteProtoToTextFile(rt, filepath));
}