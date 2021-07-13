#ifndef LIDAR_IMU_FUSION_H_
#define LIDAR_IMU_FUSION_H_

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensors.h"

using namespace std;

namespace lidar_imu {

class Fusion {
 public:
  struct Config {
    int use_n_scans = std::numeric_limits<int>::max();
    double xtol = 0.0001;
    double max_evals = 500;
  };
  struct OptData {
    vector<vector<double>> lidar;
    vector<vector<double>> vehicle;
  };
  Fusion(const Config& config);

  void optimize(Fusion::OptData* opt_data, std::vector<double>* vector_unknown);
  void LoadData(string lidar_path, string vehicle_path, OptData* opt_data);

  static Config getConfig(ros::NodeHandle* nh);

  void CsvChange();
  void test();
  void PubTF(ros::NodeHandle* nh);

 private:
  static double LidarVehicleMinimizer(const std::vector<double>& vector_unknown,
                                      std::vector<double>& grad, void* f_data);

  static pcl::PointXYZI get_global_pos(pcl::PointXYZI& pos);
  static void PointCloud2Callback(const sensor_msgs::PointCloud2& cloud);
  static void Inss2Callback(const geometry_msgs::TransformStamped& inss);

  static Transform::Matrix Vector2Matrix(std::vector<double> data);
  static void Matrix2TransformVec(Transform::Matrix matrix,
                                  std::vector<double>& rotation);
  static double CaulateError();

  static void Rvector2Quaternion(std::vector<double>& rotation_vector,
                                 std::vector<double>& quaternion_vector);

  static void Quaternion2Rotationvector(
      Eigen::Quaterniond& quaternion_vector,
      Eigen::Matrix<double, 3, 1>& rotation_vector);

  static void Matrix2Rotationvector(Eigen::Matrix<double, 4, 4> matrix,
                                    Eigen::Matrix<double, 6, 1>& vector);

  static void Rotationvector2Matrix(std::vector<double>& vector_input,
                                    Eigen::Matrix<double, 6, 1>& vector_output);

  static void Rotationmatrix2Quaternion(Eigen::Matrix3d rotation_matrix,
                                        std::vector<double>& quaternion_vector);

  inline double String2Double(const string& str) { /* return stod(str); */
    double a2;
    stringstream ss;
    ss << str;
    ss >> a2;
    return a2;
  }
  Config config_;
  vector<vector<double>> doubleArrayLidar;
  vector<vector<double>> doubleArrayVehicle;
  static ros::Publisher filt_pcl_pub;
  static ros::Publisher vehicle_pub;
  static ros::Subscriber lidar_sub;
  static ros::Subscriber inss_sub;
  static int count;
  static bool flag;
  static std::vector<geometry_msgs::TransformStamped>
      world_vehicle_transforms;  // vehicle->world
  static geometry_msgs::TransformStamped world_vehicle;
  static std::vector<geometry_msgs::TransformStamped>
      vehicle_lidar_transforms;  // lidar->vehicle
  static geometry_msgs::TransformStamped vehicle_lidar;
  static std::vector<sensor_msgs::PointCloud2> output_vector;
  static nav_msgs::Path vehicle_msg;
  static double init_x, init_y, init_z, init_qx, init_qy, init_qz, init_qw;
  static ros::Time now_time;
  static pcl::PointCloud<pcl::PointXYZI> pcl_cloud_filt_all;
  static Eigen::Matrix<double, 7, 1> expected_vector;
};

}  // namespace lidar_imu

#endif  // LIDAR_IMU_FUSION_H_
