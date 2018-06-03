// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"
#include <cmath>
#include <fstream>
#include <Eigen/Eigenvalues>

const int dx[4] = {1,-1,0,0};
const int dy[4] = {0,0,1,-1};
const double inf = (1e9);
const double L = 6;  // in dm
const int N = 60*10/((int)L);
using std::vector;

inline double sqr(double x) { return x*x; }
inline void rotate(double x, double y , double angle, double &x_, double &y_){
  x_ = x*std::cos(angle)-y*std::sin(angle);
  y_ = x*std::sin(angle)+y*std::cos(angle);
}

bool no_neighbour(int x, int y, int** v){
  for (int i = std::max(0, x-3); i <= N && i <= x+3; ++i)
    for (int j = std::max(0, y-3); j <= N && j <= y+3; ++j)
      if ((i != x || j != y) && v[i][j] != 0) return false;
  return true;
}

double get_proper_angle(int* q, int N){
  double minx_area = inf;
  double ret;
  for (int k = 0; k < 72; ++k){
    double p = CV_PI*k/36;

    double x_min = inf;
    double y_min = inf;
    double x_max = -inf;
    double y_max = -inf;
    for (int i = 0; i <= N; ++i){
      int w1 = q[i]/10000;
      int w2 = q[i]%10000;
      double x = (w1/10.*L-30.)*std::cos(p)-(w2/10.*L-30.)*std::sin(p);
      double y = (w1/10.*L-30.)*std::sin(p)+(w2/10.*L-30.)*std::cos(p);
      x_min = x_min<x?x_min:x;
      x_max = x_max>x?x_max:x;
      y_min = y_min<y?y_min:y;
      y_max = y_max>y?y_max:y;
    }
    if ((x_max-x_min)*(y_max-y_min) < minx_area){
      ret = p;
      minx_area = (x_max-x_min)*(y_max-y_min);
    }
  }
  return ret;
}

void get_rect(int* q, int N, double angle, 
              double &x_min, double &x_max,
              double &y_min, double &y_max,
              double &z_min, double &z_max,
              double** minh, double** maxh)
{
  for (int i = 0; i <= N; ++i){
    int w1 = q[i]/10000;
    int w2 = q[i]%10000;
    double x = (w1/10.*L-30.)*std::cos(angle)-(w2/10.*L-30.)*std::sin(angle);
    double y = (w1/10.*L-30.)*std::sin(angle)+(w2/10.*L-30.)*std::cos(angle);
    x_min = x_min<x?x_min:x;
    x_max = x_max>x?x_max:x;
    y_min = y_min<y?y_min:y;
    y_max = y_max>y?y_max:y;
    z_min = z_min<minh[w1][w2]?z_min:minh[w1][w2];
    z_max = z_max>maxh[w1][w2]?z_max:maxh[w1][w2];
  }
}

void add_polygon_point(interface::perception::PerceptionObstacle* obstacle,
                      double x, double y, double z,
                      const double angle,const Eigen::Vector3d& origin_point)
{
  double x_ = x*std::cos(angle)-y*std::sin(angle);
  double y_ = x*std::sin(angle)+y*std::cos(angle);
  auto* polygon_point = obstacle->add_polygon_point();
  polygon_point->set_x(x_+origin_point(0));
  polygon_point->set_y(y_+origin_point(1));
  polygon_point->set_z(z+origin_point(2));
}

void bfs(int* que, int &l, int& r, int** isobs, bool** vis, int i, int j){
  l = r = 0;
  que[l] = i*10000+j;
  vis[i][j] = true;
  while (l <= r){
    int x = que[l] / 10000;
    int y = que[l] % 10000;
    ++l;
    for (int k = 0; k < 4; ++k)
      if (x+dx[k] >= 0 && x+dx[k] <= N &&
          y+dy[k] >= 0 && y+dy[k] <= N &&
          isobs[x+dx[k]][y+dy[k]] != 0 &&
          !vis[x+dx[k]][y+dy[k]]){

        que[++r] = (x+dx[k])*10000+y+dy[k];
        vis[x+dx[k]][y+dy[k]] = true;
    }
  }
}

void Grouping(vector<Eigen::Vector3d>& points, vector<Eigen::Vector3d>** block){
  for (const Eigen::Vector3d& point: points){
      double x = point(0);
      double y = point(1);
      double z = point(2);
      if (x < -30 || x > 30 || y < -30 || y > 30){
        continue;
      }
      int idx = (int)((x+30)*10/L);
      int idy = (int)((y+30)*10/L);
      block[idx][idy].push_back(point);
  }
}

void delete_flying_points(vector<Eigen::Vector3d>** block, double** minh, double** maxh){
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      if (!block[i][j].empty()){

      std::sort(block[i][j].begin(), block[i][j].end(), [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
        return p1(2) < p2(2);
      });
      while (block[i][j].size() > 1){
        Eigen::Vector3d q1 = block[i][j].back();
        int index = block[i][j].size()-1;
        while (q1(2) - block[i][j][index](2) < 1)
          --index;

        if (block[i][j].size()-index < 15){
          block[i][j].pop_back();
          continue;
        }
        break;
      }

      std::sort(block[i][j].begin(), block[i][j].end(), [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
        return p1(2) > p2(2);
      });
      while (block[i][j].size() > 1){
        Eigen::Vector3d q1 = block[i][j].back();
        int index = block[i][j].size()-1;
        while (index >= 0 && block[i][j][index](2) - q1(2) < 1)
          --index;
        if (block[i][j].size()-index < 15){
          block[i][j].pop_back();
          continue;
        }
        break;
      }
      maxh[i][j] = block[i][j][0](2);
      minh[i][j] = block[i][j].back()(2);
    }
}

void find_obstacle(int** isobs, const double& threshold, double** minh, double** maxh){
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      if (maxh[i][j] - minh[i][j] > threshold){
        isobs[i][j] = 1;
      }
}

void delete_noneighbour_points(int** isobs){
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      if (isobs[i][j] == 1 && no_neighbour(i, j, isobs)){
        isobs[i][j] = 0;
      }
}

void Obstacle_Clustering(int ** isobs){
  int cnt = 0;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      if (isobs[i][j] == 1){
      for (int k = 0; k < 4; ++k){
        if (i+dx[k] >= 0 && i+dx[k] < N &&
            j+dy[k] >= 0 && j+dy[k] < N &&
            isobs[i+dx[k]][j+dy[k]] == 0){
          ++cnt;
          isobs[i+dx[k]][j+dy[k]] = 2;
        }
      }
    }
  std::cout << cnt << std::endl;
}

void add_object_points(interface::perception::PerceptionObstacle* obstacle,
                       int* que, int N,
                       vector<Eigen::Vector3d>** block,
                       const Eigen::Vector3d& origin_point)
{
  std::sort(que, que+N+1, [](const int& p1, const int &p2){
    int x1 = p1/10000, y1 = p1%10000;
    int x2 = p2/10000, y2 = p2%10000;
    if (x1 != x2) return x1 < x2;
    return y1 < y2;
  });

  for (int i = 0; i <= N; ++i){
    int x1 = que[i] / 10000;
    int y1 = que[i] % 10000;
    int j = i;
    while (j <= N && que[j]/10000==x1) ++j;
      --j;
    int y2 = que[j]%10000;
    i = j;
    for (int k = y1; k <= y2; ++k){
      for (const Eigen::Vector3d& point: block[x1][k]){
        auto* object_point = obstacle->add_object_points();
        object_point->set_x(point(0)+origin_point(0));
        object_point->set_y(point(1)+origin_point(1));
        object_point->set_z(point(2)+origin_point(2));
      }
    }
  }
}

void calc_feature_vector(interface::perception::PerceptionObstacle* obstacle,
                         const Eigen::Vector3d& origin_point,
                         double angle, double x_min, double x_max, double y_min, double y_max)
{
  std::vector<double> feature;
  feature.clear();
  int N = obstacle->object_points_size();
  //f1
  feature.push_back(N);
  //f2
  double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;
    t1 += sqr(point(0))+sqr(point(1));
    t2 += sqr(point(0))+sqr(point(2));
    t3 += sqr(point(1))+sqr(point(2));
    t4 += -point(0)*point(1);
    t5 += -point(0)*point(2);
    t6 += -point(1)*point(2);
  }
  double p = std::sqrt( sqr(t1)+sqr(t2)+sqr(t3)+sqr(t4)+sqr(t5)+sqr(t6)+sqr(N) );
  feature[0] /= p;
  feature.push_back(t1/p);
  feature.push_back(t2/p);
  feature.push_back(t3/p);
  feature.push_back(t4/p);
  feature.push_back(t5/p);
  feature.push_back(t6/p);
  //f3
  double x_mean = 0, y_mean = 0, z_mean = 0;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;
    x_mean += point(0);
    y_mean += point(1);
    z_mean += point(2);
  }
  x_mean /= N;
  y_mean /= N;
  z_mean /= N;
  t1 = t2 = t3 = t4 = t5 = t6 = 0;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;
    t1 += (point(0) - x_mean) * (point(0) - x_mean) ;
    t2 += (point(1) - y_mean) * (point(1) - y_mean) ;
    t3 += (point(2) - z_mean) * (point(2) - z_mean) ;
    t4 += (point(0) - x_mean) * (point(1) - y_mean) ;
    t5 += (point(0) - x_mean) * (point(2) - z_mean) ;
    t6 += (point(1) - y_mean) * (point(2) - z_mean) ;
  }
  t1 /= (N-1); t2 /= (N-1); t3 /= (N-1);
  t4 /= (N-1); t5 /= (N-1); t6 /= (N-1);
  feature.push_back(t1);
  feature.push_back(t2);
  feature.push_back(t3);
  feature.push_back(t4);
  feature.push_back(t5);
  feature.push_back(t6);
  //f4
  Eigen::Matrix3d covmatrix;  
  covmatrix << t1, t4, t5, t4, t2, t6, t5, t6, t3;  
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covmatrix);  
  if (eigenSolver.info() == Eigen::Success) {
    double* val = new double[3];
    val[0] = eigenSolver.eigenvalues()(0);
    val[1] = eigenSolver.eigenvalues()(1);
    val[2] = eigenSolver.eigenvalues()(2);
    std::sort(val, val+3, [](const double &x, const double &y){return x>y;});
    double sum = val[0]+val[1]+val[2];
    for (int i = 0; i < 3; ++i)
      val[i] /= sum;
    feature.push_back(val[0]);
    feature.push_back(val[0]-val[1]);
    feature.push_back(val[1]-val[2]);
    delete[] val;
  }
  //f7
  double* h_mean = new double[10];
  int* tot = new int[10];
  double z_min = inf, z_max = -inf;
  for (int i = 0; i < 10; ++i)
    h_mean[i] = 0, tot[i] = 0;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;
    z_min = z_min<point(2)?z_min:point(2);
    z_max = z_max>point(2)?z_max:point(2);
    
    double x = point(0), y = point(1);
    rotate(x, y, angle, point(0), point(1));

    if (x_max-x_min > y_max-y_min){
      int index = (int)((point(0)-x_min)/(x_max-x_min)*10);
      if (index < 0) index = 0;
      if (index >= 10) index = 9;
      h_mean[index] += point(2);
      tot[index]++;
    } else
    {
      int index = (int)((point(1)-y_min)/(y_max-y_min)*10);
      if (index < 0) index = 0;
      if (index >= 10) index = 9;
      h_mean[index] += point(2);
      tot[index]++;
    }
  }
  for (int i = 0; i < 10; ++i){
    if (tot[i] > 0)
      h_mean[i] /= tot[i];
    feature.push_back(h_mean[i]);
  }
  delete[] h_mean;
  delete[] tot;
  
  //f8
  if (x_max-x_min > y_max-y_min){
    feature.push_back( (y_max-y_min)/(x_max-x_min) );
    feature.push_back( (y_max-y_min)/(z_max-z_min) );
  } else
  {
    feature.push_back( (x_max-x_min)/(y_max-y_min) );
    feature.push_back( (x_max-x_min)/(z_max-z_min) );
  }
  double cx1 = (x_max+x_min)/2, cx2, cx;
  double cy1 = (y_max+y_min)/2, cy2, cy;
  rotate(cx1, cy1, -angle, cx, cy);
  feature.push_back( std::sqrt( sqr(cx) + sqr(cy) ) );
  feature.push_back(std::atan2(cy, cx));

  if (x_max-x_min > y_max-y_min){
    cx = x_min; cy = y_min;
    rotate(cx, cy, -angle, cx1, cy1);
    cx = x_max; cy = y_min;
    rotate(cx, cy, -angle, cx2, cy2);
  } else
  {
    cx = x_min; cy = y_min;
    rotate(cx, cy, -angle, cx1, cy1);
    cx = x_min; cy = y_max;
    rotate(cx, cy, -angle, cx2, cy2);
  }
  double beta = 0;
  if (std::fabs(cx1-cx2) > 10*std::fabs(cy1-cy2)){
    beta = std::atan( (cy1-cy2) / (cx1-cx2) );
    if (beta < 0){
      beta = -CV_PI - beta;
    } else
      beta = CV_PI - beta;
  } else
  {
    beta = std::atan( (cx1-cx2) / (cy1-cy2) );
  }
  feature.push_back(beta);
  //output

  std::ofstream out("/home/ycdfwzy/Desktop/feature.txt", std::ios::app);
  if (out.is_open()){
    
    if (obstacle->type() == interface::perception::ObjectType::CAR){
      out << "1 ";
    } else
      out << "-1 ";
  
    int tot = feature.size();
    p = 0;
    for (int i = 0; i < tot; ++i){
      p += sqr(feature[i]);
    }
    p = std::sqrt(p);

    for (int i = 0; i < tot; ++i){
      feature[i] /= p;
      out << i << ":" << feature[i] << " ";
    }
    //for (const double &x: feature)
    //  out << x << " ";
    out << std::endl;
    out.close();
  }
}

interface::perception::PerceptionObstacles Perception::RunPerception(
    const PointCloud& pointcloud, const utils::Optional<cv::Mat>& image) {
  interface::perception::PerceptionObstacles perception_result;

  PointCloud pc(pointcloud);
  separate_ground_points(pc);

  double threshold = 0.1;
  /*---------------allocate arrays---------------*/
  double **maxh = new double*[N+10];
  double **minh = new double*[N+10];
  int **isobs = new int*[N+10];
  bool** vis = new bool*[N+10];
  int* que = new int[(N+10)*(N+10)];

  std::vector<Eigen::Vector3d>** block
    = new std::vector<Eigen::Vector3d>*[N+10];

  for (int i = 0; i < N+10; ++i){
    maxh[i] = new double[N+10];
    minh[i] = new double[N+10];
    isobs[i] = new int[N+10];
    vis[i] = new bool[N+10];
    block[i] = new std::vector<Eigen::Vector3d>[N+10];
    std::memset(maxh[i], -63, sizeof(double)*(N+10));
    std::memset(minh[i], 63, sizeof(double)*(N+10));
    std::memset(isobs[i], 0, sizeof(int)*(N+10));
    std::memset(vis[i], false, sizeof(bool)*(N+10));
    for (int j = 0; j < N+10; ++j)
      block[i][j].clear();
  }
  /*--------------------------------------------*/

  std::vector<Eigen::Vector3d>& points = pc.points;

  /*----from local coordinate system to WCS----*/
  Eigen::Vector3d origin_point(0, 0, 0);
  origin_point = pc.rotation * origin_point + pc.translation;
  for (Eigen::Vector3d& point: points){
    point = pc.rotation * point + pc.translation;
    point = point - origin_point;
  }
  /*--------------------------------------------*/
  Grouping(points, block);
  delete_flying_points(block, minh, maxh);
  find_obstacle(isobs, threshold, minh, maxh);
  delete_noneighbour_points(isobs);
  Obstacle_Clustering(isobs);

  /*----------------add obstacle----------------*/
  int cnt = 0;
  for (int i = 0; i < N; ++i){
    for (int j = 0; j < N; ++j){

      if (isobs[i][j] != 0 && !vis[i][j]){
        //bfs
        ++cnt;
        int l, r;
        bfs(que, l, r, isobs, vis, i, j);

        double angle = get_proper_angle(que, r);

        double x_min = inf;
        double y_min = inf;
        double z_min = inf;
        double x_max = -inf;
        double y_max = -inf;
        double z_max = -inf;
        get_rect(que, r, angle, x_min, x_max, y_min, y_max, z_min, z_max, minh, maxh);

        auto* obstacle = perception_result.add_obstacle();
        obstacle->set_type(interface::perception::ObjectType::CAR);
        add_polygon_point(obstacle, x_min, y_min, z_min, -angle, origin_point);
        add_polygon_point(obstacle, x_min, y_max, z_min, -angle, origin_point);
        add_polygon_point(obstacle, x_max, y_max, z_min, -angle, origin_point);
        add_polygon_point(obstacle, x_max, y_min, z_min, -angle, origin_point);
        obstacle->set_height(z_max-z_min);
        obstacle->set_id("car"+std::to_string(cnt));

        add_object_points(obstacle, que, r, block, origin_point);
        calc_feature_vector(obstacle, origin_point, angle, x_min, x_max, y_min, y_max);
      }
    }
  }
  std::cout << cnt << std::endl;
  /*--------------------------------------------*/

  /*-----------------free space-----------------*/
  delete[] que;
  for (int i = 0; i < N+10; ++i){
    delete[] maxh[i];
    delete[] minh[i];
    delete[] isobs[i];
    delete[] vis[i];
    for (int j = 0; j < N+10; ++j)
      block[i][j].clear();
    delete[] block[i];
  }
  delete[] maxh;
  delete[] minh;
  delete[] isobs;
  delete[] vis;
  delete[] block;
  /*--------------------------------------------*/
  /*
  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::CAR);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(976.05);
      polygon_point->set_y(1079.60);
      polygon_point->set_z(-8.13);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(974.76);
      polygon_point->set_y(1087.13);
      polygon_point->set_z(-8.13);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(972.22);
      polygon_point->set_y(1086.69);
      polygon_point->set_z(-8.13);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(973.52);
      polygon_point->set_y(1079.16);
      polygon_point->set_z(-8.13);
    }
    obstacle->set_height(2.69);
    obstacle->set_id("c83");
  }
  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::CAR);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(972.10);
      polygon_point->set_y(1084.33);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(971.20);
      polygon_point->set_y(1088.79);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(969.00);
      polygon_point->set_y(1088.34);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(969.91);
      polygon_point->set_y(1083.89);
      polygon_point->set_z(-8.18);
    }
    obstacle->set_height(1.56);
    obstacle->set_id("c84");
  }

  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::UNKNOWN_TYPE);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(967.10);
      polygon_point->set_y(1084.3295967224144);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(965.19);
      polygon_point->set_y(1088.79);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(964.01);
      polygon_point->set_y(1088.34);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(964.91);
      polygon_point->set_y(1083.89);
      polygon_point->set_z(-8.18);
    }
    obstacle->set_height(1.56);
    obstacle->set_id("c88");
  }
  */
  if (image) {
    // Remove me if you don't want to pause the program every time.
    cv::namedWindow("camera");
    imshow("camera", *image);
    cv::waitKey(0);
  }

  LOG(INFO) << "Perception done.";
  return perception_result;
}

void separate_ground_points(PointCloud& pointcloud){
  //showPointCloud(pointcloud);
  vector<Eigen::Vector3d>& points = pointcloud.points;
  int N = points.size();
  sort(points.begin(), points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b){
    return atan2(a(1),a(0)) < atan2(b(1), b(0));
  });
  //ofstream out("sorted.txt");
  const double eps = CV_PI/1000;
  double low_range = -CV_PI;
  
  vector<Eigen::Vector3d> points_; points_.clear();
  for (int i = 0, j = 0; i < 2000; ++i, low_range += eps){
    vector<Eigen::Vector3d> line_points;
    line_points.clear();
    while (j < N){
      double angle = atan2(points[j](1), points[j](0));
      if (angle >= low_range && angle < low_range+eps){
        line_points.push_back(points[j++]);
        continue;
      }
      break;
    }
    detect_ground_points(line_points);
    points_.insert(points_.begin(), line_points.begin(), line_points.end());
  }
  //out.close();

  points.clear();
  //cout << points.size() << endl;
  points.insert(points.begin(), points_.begin(), points_.end());
  //showPointCloud(pointcloud);
}

void detect_ground_points(vector<Eigen::Vector3d>& points){
  sort(points.begin(), points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b){
    double d1 = sqrt(a(0)*a(0)+a(1)*a(1));
    double d2 = sqrt(b(0)*b(0)+b(1)*b(1));
    return atan2(a(2)-2, d1) < atan2(b(2)-2, d2);
  });

  int N = points.size();
  Eigen::Vector3d start_point(Eigen::Vector3d::Zero());
  double alpha = CV_PI/4;
  double h_min = 0.4;
  vector<Eigen::Vector3d> ground;
  ground.clear();

  for (int i = 0; i < N; ++i){
    Eigen::Vector3d pre_point(start_point);
    while (i < N){
      double d1 = sqrt(pre_point(0)*pre_point(0)+pre_point(1)*pre_point(1));
      double d2 = sqrt(points[i](0)*points[i](0)+points[i](1)*points[i](1));
      double D1 = sqrt(pre_point(0)*pre_point(0)+pre_point(1)*pre_point(1)+pre_point(2)*pre_point(2));
      double D2 = sqrt(points[i](0)*points[i](0)+points[i](1)*points[i](1)+points[i](2)*points[i](2));
      if ( D1 > D2 || atan2(points[i](2)-pre_point(2) ,d2-d1) > alpha || points[i](2)-pre_point(2) > h_min ){
        int j = i+1;
        while (j < N && points[j](2)-pre_point(2) > h_min)
          ++j;
        i = j;
        if (j < N)
          start_point = points[j];
        break;
      }
      ground.push_back(points[i]);
      pre_point = points[i++];
    }
  }
  /*
  points.clear();
  while (!ground.empty()){
    points.push_back(ground.back());
    ground.pop_back();
  }
  */
  for (int i = 0; i < points.size(); ++i){
    for (const Eigen::Vector3d& p: ground){
      if (p == points[i]){
        points.erase(points.begin()+i);
        i--;
        break;
      }
    }
  }
}