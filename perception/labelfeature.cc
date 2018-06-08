#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Eigenvalues>

#include "common/proto/object_labeling_3d.pb.h"
#include "common/proto/perception_evaluation.pb.h"
#include "common/utils/evaluation/grading.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/hungarian/hungarian_sparse.h"
#include "common/utils/math/transform/transform.h"
#include "common/utils/math/vec2d.h"
#include "common/utils/strings/format.h"
#include "homework2/pointcloud.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "common/utils/common/optional.h"

using interface::object_labeling::ObjectLabel;
using interface::object_labeling::ObjectLabels;
using interface::perception::EffectivePolygonInfo;
using interface::perception::ObjectType;
using interface::perception::PerceptionObstacles;
using interface::perception::PerceptionEvaluationResult;
using interface::perception::PerceptionFrameResult;
using interface::geometry::Polygon;
using interface::geometry::Point3D;
using std::vector;

const std::string path_prefix = "/home/ycdfwzy/autonomous_driving/PublicCourse/perception_project/20171229_nanian_130255";
const double inf = 1e9;

inline double sqr(double x) { return x*x; }
inline void rotate(double x, double y , double angle, double &x_, double &y_){
  x_ = x*std::cos(angle)-y*std::sin(angle);
  y_ = x*std::sin(angle)+y*std::cos(angle);
}
inline void swap_point(Point3D* p1, Point3D* p2){
    Point3D* p3 = new Point3D;
    p3->set_x(p1->x()); p3->set_y(p1->y()); p3->set_z(p1->z());

    p1->set_x(p2->x()); p1->set_y(p2->y()); p1->set_z(p2->z());
    p2->set_x(p3->x()); p2->set_y(p3->y()); p2->set_z(p3->z());
    delete p3;
}

inline double distance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
  return std::sqrt( sqr(p1(0)-p2(0)) + sqr(p1(1)-p2(1)) + sqr(p1(2)-p2(2)) );
}


inline double mult(Eigen::Vector2d A, Eigen::Vector2d B){
  if (std::fabs(A(0)*B(1)-A(1)*B(0)) < 1e-6) return 0;
  if (A(0)*B(1)-A(1)*B(0) > 0) return 1;
  return -1;
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

bool check_in_label(Eigen::Vector3d& point_in_world, const ObjectLabel& object){
      Eigen::Vector2d point;
      point << point_in_world(0), point_in_world(1);
      if (point_in_world(2) < object.polygon().point()[0].z() ||
        point_in_world(2) > object.polygon().point()[0].z()+object.height())
        return false;
      Eigen::Vector2d P1, P2, P3, P4;
      P1 << object.polygon().point()[0].x(), object.polygon().point()[0].y();
      P2 << object.polygon().point()[1].x(), object.polygon().point()[1].y();
      P3 << object.polygon().point()[2].x(), object.polygon().point()[2].y();
      P4 << object.polygon().point()[3].x(), object.polygon().point()[3].y();
      Eigen::Vector2d v1 = P2-P1;
      Eigen::Vector2d v2 = P4-P1;
      Eigen::Vector2d v3 = point-P1;
      Eigen::Vector2d v4 = P2-P3;
      Eigen::Vector2d v5 = P4-P3;
      Eigen::Vector2d v6 = point-P3;
      if (mult(v1, v2)*mult(v1, v3) > -(1e-6) && mult(v2, v1)*mult(v2, v3) > -(1e-6) &&
        mult(v4, v5)*mult(v4, v6) > -(1e-6) && mult(v5, v4)*mult(v5, v6) > -(1e-6))
        return true;
      return false;
}

void Lalonde_features(std::vector<Eigen::Vector3d>& points,
                      std::vector<double>& L1,
                      std::vector<double>& L2,
                      std::vector<double>& L3)
{

  if (points.empty()){
    std::cout << "No neighbour" << std::endl;
    return;
  }
  Eigen::Vector3d means(0, 0, 0);
  for (auto& point: points)
    means = means + point;
  means = means / points.size();

  Eigen::Matrix3d mat;
  mat << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  for (auto& point: points){
    Eigen::Vector3d p = point-means;
    mat = mat+p*p.transpose();
  }
  mat = mat / points.size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(mat);
  if (eigenSolver.info() == Eigen::Success) {
      double* val = new double[3];
      val[0] = eigenSolver.eigenvalues()(0);
      val[1] = eigenSolver.eigenvalues()(1);
      val[2] = eigenSolver.eigenvalues()(2);
      std::sort(val, val+3, [](const double &x, const double &y){return x>y;});
      double sum = val[0]+val[0]-val[2];
      L1.push_back(val[0]/sum);
      L2.push_back((val[0]-val[1])/sum);
      L3.push_back((val[1]-val[2])/sum);
      delete[] val;
  }
}

void divide_bins(std::vector<double>& feature, std::vector<double>& L, int bins = 4){
  std::sort(L.begin(), L.end());
  for (int i = 0, j = 0; i < bins; ++i){
    double tot = 0;
    while (j < L.size() && bins*L[j] < (i+1)){
      tot += 1;
      j ++;
    }
    feature.emplace_back(tot);
  }
}

void noramlize(std::vector<double>& feature){
  int N = feature.size();
  double sqrL = 0;
  for (int i = 0; i < N; ++i)
    sqrL = sqr(feature[i]);
  double L = std::sqrt(sqrL);
  for (int i = 0; i < N; ++i)
    feature[i] /= L;
}

void calc_feature_vector(interface::perception::PerceptionObstacle* obstacle,
                         const Eigen::Vector3d& origin_point,
                         double angle, double x_min, double x_max, double y_min, double y_max)
{
  std::vector<double> feature;
  feature.clear();
  int N = obstacle->object_points_size();
  if (N == 0) return;

  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;
    points.emplace_back(point);
  }

  /*-------------Lalonde features-------------*/
  std::vector<double> L1, L2, L3;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;

    std::sort(points.begin(), points.end(), [&point](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
      //return p1.distance(point) < p2.distance(point);
      return distance(p1, point) < distance(p2, point);
    });

    std::vector<Eigen::Vector3d> neighbours;
    for (int j = 1; j < points.size() && j <= 20; ++j)
      neighbours.push_back(points[j]);
    Lalonde_features(neighbours, L1, L2, L3);
  }
  divide_bins(feature, L1);
  divide_bins(feature, L2);
  divide_bins(feature, L3);
  /*-----------------------------------------*/

  /*------------Anguelov features------------*/
  std::vector<double> A1, A2, A3;
  for (int i = 0; i < N; ++i){
    Eigen::Vector3d point(obstacle->object_points(i).x(),
                          obstacle->object_points(i).y(),
                          obstacle->object_points(i).z());
    point = point - origin_point;

    double* tot = new double[3];
    for (int j = 0; j < 3; ++j)
      tot[j] = 0;
    for (auto& p: points)
    if (sqr(p(0)-point(0))+sqr(p(1)-point(1)) <= 0.01){
      if (p(2) >= point(2)-2./2 && p(2) < point(2)-2./2+2./3){
        ++tot[0];
      } else
      if (p(2) >= point(2)-2./2+2./3 && p(2) < point(2)+2./2-2./3){
        ++tot[1];
      } else
      if(p(2) >= point(2)+2./2-2./3 && p(2) <= point(2)+2./2){
        ++tot[2];
      }
    }
    double sum = tot[0]+tot[1]+tot[2];
    A1.emplace_back(tot[0]/sum);
    A2.emplace_back(tot[1]/sum);
    A3.emplace_back(tot[2]/sum);
    delete[] tot;
  }
  divide_bins(feature, A1);
  divide_bins(feature, A2);
  divide_bins(feature, A3);
  /*-----------------------------------------*/

  // //f1
  // feature.push_back(N);
  // //f2
  // double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0;
  // for (int i = 0; i < N; ++i){
  //   Eigen::Vector3d point(obstacle->object_points(i).x(),
  //                         obstacle->object_points(i).y(),
  //                         obstacle->object_points(i).z());
  //   point = point - origin_point;
  //   t1 += sqr(point(0))+sqr(point(1));
  //   t2 += sqr(point(0))+sqr(point(2));
  //   t3 += sqr(point(1))+sqr(point(2));
  //   t4 += -point(0)*point(1);
  //   t5 += -point(0)*point(2);
  //   t6 += -point(1)*point(2);
  // }
  // double p = std::sqrt( sqr(t1)+sqr(t2)+sqr(t3)+sqr(t4)+sqr(t5)+sqr(t6)+sqr(N) );
  // feature[0] /= p;
  // feature.push_back(t1/p);
  // feature.push_back(t2/p);
  // feature.push_back(t3/p);
  // feature.push_back(t4/p);
  // feature.push_back(t5/p);
  // feature.push_back(t6/p);
  // //f3
  // double x_mean = 0, y_mean = 0, z_mean = 0;
  // for (int i = 0; i < N; ++i){
  //   Eigen::Vector3d point(obstacle->object_points(i).x(),
  //                         obstacle->object_points(i).y(),
  //                         obstacle->object_points(i).z());
  //   point = point - origin_point;
  //   x_mean += point(0);
  //   y_mean += point(1);
  //   z_mean += point(2);
  // }
  // x_mean /= N;
  // y_mean /= N;
  // z_mean /= N;
  // t1 = t2 = t3 = t4 = t5 = t6 = 0;
  // for (int i = 0; i < N; ++i){
  //   Eigen::Vector3d point(obstacle->object_points(i).x(),
  //                         obstacle->object_points(i).y(),
  //                         obstacle->object_points(i).z());
  //   point = point - origin_point;
  //   t1 += (point(0) - x_mean) * (point(0) - x_mean) ;
  //   t2 += (point(1) - y_mean) * (point(1) - y_mean) ;
  //   t3 += (point(2) - z_mean) * (point(2) - z_mean) ;
  //   t4 += (point(0) - x_mean) * (point(1) - y_mean) ;
  //   t5 += (point(0) - x_mean) * (point(2) - z_mean) ;
  //   t6 += (point(1) - y_mean) * (point(2) - z_mean) ;
  // }
  // t1 /= (N-1); t2 /= (N-1); t3 /= (N-1);
  // t4 /= (N-1); t5 /= (N-1); t6 /= (N-1);
  // feature.push_back(t1);
  // feature.push_back(t2);
  // feature.push_back(t3);
  // feature.push_back(t4);
  // feature.push_back(t5);
  // feature.push_back(t6);
  // //f4
  // Eigen::Matrix3d covmatrix;  
  // covmatrix << t1, t4, t5, t4, t2, t6, t5, t6, t3;  
  // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covmatrix);  
  // if (eigenSolver.info() == Eigen::Success) {
  //   double* val = new double[3];
  //   val[0] = eigenSolver.eigenvalues()(0);
  //   val[1] = eigenSolver.eigenvalues()(1);
  //   val[2] = eigenSolver.eigenvalues()(2);
  //   std::sort(val, val+3, [](const double &x, const double &y){return x>y;});
  //   double sum = val[0]+val[1]+val[2];
  //   for (int i = 0; i < 3; ++i)
  //     val[i] /= sum;
  //   feature.push_back(val[0]);
  //   feature.push_back(val[0]-val[1]);
  //   feature.push_back(val[1]-val[2]);
  //   delete[] val;
  // }
  // //f7
  // double* h_mean = new double[10];
  // int* tot = new int[10];
  // double z_min = inf, z_max = -inf;
  // for (int i = 0; i < 10; ++i)
  //   h_mean[i] = 0, tot[i] = 0;
  // for (int i = 0; i < N; ++i){
  //   Eigen::Vector3d point(obstacle->object_points(i).x(),
  //                         obstacle->object_points(i).y(),
  //                         obstacle->object_points(i).z());
  //   point = point - origin_point;
  //   z_min = z_min<point(2)?z_min:point(2);
  //   z_max = z_max>point(2)?z_max:point(2);
    
  //   double x = point(0), y = point(1);
  //   rotate(x, y, angle, point(0), point(1));

  //   if (x_max-x_min > y_max-y_min){
  //     int index = (int)((point(0)-x_min)/(x_max-x_min)*10);
  //     if (index < 0) index = 0;
  //     if (index >= 10) index = 9;
  //     h_mean[index] += point(2);
  //     tot[index]++;
  //   } else
  //   {
  //     int index = (int)((point(1)-y_min)/(y_max-y_min)*10);
  //     if (index < 0) index = 0;
  //     if (index >= 10) index = 9;
  //     h_mean[index] += point(2);
  //     tot[index]++;
  //   }
  // }
  // for (int i = 0; i < 10; ++i){
  //   if (tot[i] > 0)
  //     h_mean[i] /= tot[i];
  //   feature.push_back(h_mean[i]);
  // }
  // delete[] h_mean;
  // delete[] tot;
  
  // //f8
  // if (x_max-x_min > y_max-y_min){
  //   feature.push_back( (y_max-y_min)/(x_max-x_min) );
  //   feature.push_back( (y_max-y_min)/(z_max-z_min) );
  // } else
  // {
  //   feature.push_back( (x_max-x_min)/(y_max-y_min) );
  //   feature.push_back( (x_max-x_min)/(z_max-z_min) );
  // }
  // double cx1 = (x_max+x_min)/2, cx2, cx;
  // double cy1 = (y_max+y_min)/2, cy2, cy;
  // rotate(cx1, cy1, -angle, cx, cy);
  // feature.push_back( std::sqrt( sqr(cx) + sqr(cy) ) );
  // feature.push_back(std::atan2(cy, cx));

  // if (x_max-x_min > y_max-y_min){
  //   cx = x_min; cy = y_min;
  //   rotate(cx, cy, -angle, cx1, cy1);
  //   cx = x_max; cy = y_min;
  //   rotate(cx, cy, -angle, cx2, cy2);
  // } else
  // {
  //   cx = x_min; cy = y_min;
  //   rotate(cx, cy, -angle, cx1, cy1);
  //   cx = x_min; cy = y_max;
  //   rotate(cx, cy, -angle, cx2, cy2);
  // }
  // double beta = 0;
  // if (std::fabs(cx1-cx2) > 10*std::fabs(cy1-cy2)){
  //   beta = std::atan( (cy1-cy2) / (cx1-cx2) );
  //   if (beta < 0){
  //     beta = -CV_PI - beta;
  //   } else
  //     beta = CV_PI - beta;
  // } else
  // {
  //   beta = std::atan( (cx1-cx2) / (cy1-cy2) );
  // }
  // feature.push_back(beta);
  // //output

  // for (double x: feature){
  //   if (std::isnan(x))
  //       return;
  // }
  std::ofstream out("/home/ycdfwzy/Desktop/allfeature.txt", std::ios::app);
  if (out.is_open()){
    
    if (obstacle->type() == interface::perception::ObjectType::CAR){
      out << "1 ";
      std::cout << "1 ";
    } else{
      out << "-1 ";
      std::cout << "-1 ";
    }
  
    int tot = feature.size();
    double p = 0;
    for (int i = 0; i < tot; ++i){
      p += sqr(feature[i]);
    }
    p = std::sqrt(p);

    for (int i = 0; i < tot; ++i){
      std::cout << i << ":" << feature[i] << " ";
      feature[i] /= p;
      out << i << ":" << feature[i] << " ";
    }
    //for (const double &x: feature)
    //  out << x << " ";
    out << std::endl;
    std::cout << std::endl;
    out.close();
  }
}

double get_angle(const Polygon& poly, Eigen::Vector3d& origin_point){
	double x[poly.point_size()];
	double y[poly.point_size()];
	double x_[poly.point_size()];
	double y_[poly.point_size()];

	for (int i = 0; i < poly.point_size(); ++i){
		x[i] = poly.point()[i].x()-origin_point(0);
		y[i] = poly.point()[i].y()-origin_point(1);
	}
	for (int k = 0; k < 72; ++k){
		double angle = CV_PI*k/36;
		double x_min = inf, x_max = -inf;
		double y_min = inf, y_max = -inf;
		for (int i = 0; i < poly.point_size(); ++i){
			rotate(x[i], y[i], angle, x_[i], y_[i]);
			x_min = x_min<x_[i]?x_min:x_[i];
			x_max = x_max>x_[i]?x_max:x_[i];
			y_min = y_min<y_[i]?y_min:y_[i];
			y_max = y_max>y_[i]?y_max:y_[i];
	    }
	    if (x_max-x_min < 1e-4 && y_max-y_min < 1e-4){
	    	return angle;
	    }
	}
    return 0;
}

void  get_file(){
    std::ofstream out("/home/ycdfwzy/Desktop/allfeature.txt", std::ios::app);
    out << std::endl;
    out.close();

	std::string label_path_suffix = "/label/VelodyneDevice32c/";
	std::string select_path_suffix = "/select/VelodyneDevice32c/";
	for (int I = 0; I < 500; ++I){
		std::string label_file = path_prefix+label_path_suffix+std::to_string(I)+".label";
		std::string pointcloud_file = path_prefix+select_path_suffix+std::to_string(I)+".txt";

		const PointCloud pointcloud = ReadPointCloudFromTextFile(pointcloud_file);
		PointCloud pc(pointcloud);
    separate_ground_points(pc);
		for (Eigen::Vector3d& point: pc.points){
			point = pc.rotation * point + pc.translation;
		}

		ObjectLabels object_labels;
        CHECK(file::ReadFileToProto(label_file, &object_labels));
        
        int N = object_labels.object_size();
        interface::perception::PerceptionObstacles perception_result;
        for (int i = 0; i < N; ++i){
          ObjectLabel* object_label = object_labels.mutable_object(i);
            
          auto* obstacle = perception_result.add_obstacle();
          obstacle->set_type(object_label->type());
        	obstacle->set_id(object_label->id());
        	obstacle->set_height(object_label->height());

        	for (int j = 0; j < object_label->polygon().point_size(); ++j){
        		auto* polygon_point = obstacle->add_polygon_point();
  				  polygon_point->set_x(object_label->polygon().point(j).x());
  				  polygon_point->set_y(object_label->polygon().point(j).y());
 				    polygon_point->set_z(object_label->polygon().point(j).z());
        	}
          // std::sort(object_label->mutable_polygon()->mutable_point()->begin(),
          //           object_label->mutable_polygon()->mutable_point()->end(),
          //           [](const Point3D& p1, const Point3D& p2){
          //               if (std::fabs(p1.x()-p2.x()) < 1e-4)
          //                   return p1.y() < p2.y();
          //               return p1.x() < p2.x();
          //           });
          // //object_label->mutable_polygon()->mutable_point()
          // swap_point(object_label->mutable_polygon()->mutable_point(0),
          //            object_label->mutable_polygon()->mutable_point(1));

        	for (Eigen::Vector3d& point: pc.points){
        		if ( check_in_label(point, *object_label) ){
        			auto* object_point = obstacle->add_object_points();
        			object_point->set_x(point(0));
        			object_point->set_y(point(1));
        			object_point->set_z(point(2));
        		}
        	}
          if (obstacle->object_points_size() == 0)
            continue;
          //std::cout << obstacle->object_points_size() << std::endl;

        	double angle = get_angle(object_label->polygon(), pc.translation);
        	double x_min = inf, x_max = -inf;
			    double y_min = inf, y_max = -inf;
			    for (int j = 0; j < object_label->polygon().point_size(); ++j){
				    double x_, y_;
				    rotate(object_label->polygon().point()[j].x(), object_label->polygon().point()[j].y(), angle, x_, y_);
				    x_min = x_min<x_?x_min:x_;
				    x_max = x_max>x_?x_max:x_;
				    y_min = y_min<y_?y_min:y_;
				    y_max = y_max>y_?y_max:y_;
	    	  }
          // if (obstacle->object_points_size() > 10000){
          //     std::cout << "    " << pc.points.size() << std::endl;
          //     std::cout << "    " << I << std::endl;
          //     std::cout << "    " << i << std::endl;
          //     std::cout << "    " << object_label->id() << std::endl;
          // }
          // std::cout << obstacle->object_points_size() << std::endl;
        	calc_feature_vector(obstacle, pc.translation, angle, x_min, x_max, y_min, y_max);
        }
	}
}

int main(){
	get_file();
	return 0;
}