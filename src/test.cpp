#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fusion.h"

using namespace Eigen;
using namespace std;

namespace lidar_imu {

ros::Publisher Fusion::filt_pcl_pub;
ros::Publisher Fusion::vehicle_pub;
ros::Subscriber Fusion::lidar_sub;
ros::Subscriber Fusion::inss_sub;
// ros::Subscriber Fusion::filt_lidar_sub;
int Fusion::count = 0;
double Fusion::init_x, Fusion::init_y, Fusion::init_z = 0.;
ros::Time Fusion::now_time;
vector<geometry_msgs::TransformStamped> Fusion::world_vehicle_transforms;
// geometry_msgs::TransformStamped Fusion::world_vehicle;
vector<geometry_msgs::TransformStamped> Fusion::vehicle_lidar_transforms;
geometry_msgs::TransformStamped Fusion::vehicle_lidar;
nav_msgs::Path Fusion::vehicle_msg;
std::vector<sensor_msgs::PointCloud2> Fusion::output_vector;
pcl::PointCloud<pcl::PointXYZ> Fusion::pcl_cloud_filt_all;
Eigen::Matrix<double, 7, 1> Fusion::expected_vector;
geometry_msgs::TransformStamped Fusion::world_vehicle;

Fusion::Fusion(const Config& config) : config_(config) {}

Fusion::Config Fusion::getConfig(ros::NodeHandle* nh) {
  Fusion::Config config;
  nh->param("use_n_scans", config.use_n_scans, config.use_n_scans);
  nh->param("xtol", config.xtol, config.xtol);
  nh->param("max_evals", config.max_evals, config.max_evals);
  return config;
}

// 读取csv文件，存储到vector中
void Fusion::LoadData(string lidar_path, string vehicle_path,
                      OptData* opt_data) {
  // 读取lidar_key_pose_new.csv
  ifstream fin_lidar(lidar_path, ios::in);
  string lineStr;
  std::getline(fin_lidar, lineStr);  // 跳过第一行
  while (getline(fin_lidar, lineStr)) {
    istringstream ss_lidar(lineStr);
    string str;
    vector<double> lineArrayLidar;
    while (getline(ss_lidar, str, ',')) {
      lineArrayLidar.push_back(String2Double(str));
    }
    doubleArrayLidar.push_back(lineArrayLidar);
  }
  // 读取vehicle_key_pose_new.csv
  ifstream fin_vehicle(vehicle_path, ios::in);
  std::getline(fin_vehicle, lineStr);
  while (getline(fin_vehicle, lineStr)) {
    istringstream ss_vehicle(lineStr);
    string str;
    vector<double> lineArrayVehicle;
    while (getline(ss_vehicle, str, ',')) {
      lineArrayVehicle.push_back(String2Double(str));
    }
    doubleArrayVehicle.push_back(lineArrayVehicle);
  }

  opt_data->lidar = doubleArrayLidar;
  opt_data->vehicle = doubleArrayVehicle;
}

// 旋转矩阵转四元数(正确)
void Fusion::Rotationmatrix2Quaternion(Eigen::Matrix3d rotation_matrix,
                                       std::vector<double>& quaternion_vector) {
  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
  q.normalize();
  quaternion_vector = {q.x(), q.y(), q.z(), q.w()};
}

pcl::PointXYZ Fusion::get_global_pos(pcl::PointXYZ& pos) {
  // std::cout << "pos in laser"
  //           << "  x:" << pos.x << "  y:" << pos.z << "  z:" << pos.y
  //           << std::endl;
  pcl::PointXYZ res;
  Eigen::Quaterniond q_w2i(
      world_vehicle.transform.rotation.w, world_vehicle.transform.rotation.x,
      world_vehicle.transform.rotation.y, world_vehicle.transform.rotation.z),
      q_i2l(expected_vector[6], expected_vector[3], expected_vector[4],
            expected_vector[5]);
  q_w2i.normalize();
  q_i2l.normalize();
  Eigen::Vector3d t_w2i(world_vehicle.transform.translation.x,
                        world_vehicle.transform.translation.y,
                        world_vehicle.transform.translation.z),
      t_i2l(expected_vector[0], expected_vector[1], expected_vector[2]);
  Eigen::Vector3d p_l(pos.x, pos.y, pos.z);

  Eigen::Isometry3d T_w2i(q_w2i), T_i2l(q_i2l);
  T_w2i.pretranslate(t_w2i);
  T_i2l.pretranslate(t_i2l);

  Eigen::Vector3d p_l_w = T_w2i * T_i2l * p_l;
  // std::cout << p_l_w.transpose() << std::endl;
  res.x = p_l_w[0];
  res.y = p_l_w[1];
  res.z = p_l_w[2];

  return res;
}

void Fusion::PointCloud2Callback(const sensor_msgs::PointCloud2& cloud) {
  //   std::cout << "PointCloud2Callback: " << count << std::endl;
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_filt;
  for (int i = 0; i < pcl_cloud.points.size(); i++) {
    pcl::PointXYZ point = pcl_cloud.points[i];
    if (point.x < 20 && point.x > -20 && point.y < 5 && point.y > -5 &&
        point.z < 0.5) {
      point = get_global_pos(point);
      pcl_cloud_filt.points.push_back(point);
    }
  }
  pcl_cloud_filt_all += pcl_cloud_filt;
  // std::cout << pcl_cloud_filt_all.points.size() << " "
  //           << pcl_cloud_filt.points.size() << " " <<
  //           pcl_cloud_filt_all.width
  //           << " " << pcl_cloud_filt_all.height << " " << std::endl;

  sensor_msgs::PointCloud2 filt_cloud;
  pcl::PCLPointCloud2 pcl_pc2_new;
  pcl::toPCLPointCloud2(pcl_cloud_filt_all, pcl_pc2_new);
  pcl_conversions::moveFromPCL(pcl_pc2_new, filt_cloud);
  // std::cout << filt_cloud.height << " " << filt_cloud.width << std::endl;

  filt_cloud.header.stamp = now_time;
  filt_cloud.header.frame_id = "lidar" + to_string(count);
  filt_pcl_pub.publish(filt_cloud);
  // output_vector.push_back(filt_cloud);
  // for (auto i : output_vector) {
  //   filt_pcl_pub.publish(i);
  // }

  // ---
  Eigen::Matrix<double, 6, 1> vector_transform;
  vector_transform << 0.147015, 0.500000, 1.003473, 0.011452, 0.048167,
      1.557202;
  // 要验证的变换(lidar->vehicle)
  Transform::Matrix matrix_transform =
      Transform::exp(vector_transform.cast<float>()).matrix();

  Eigen::Matrix<double, 4, 4> expected_matrix = matrix_transform.cast<double>();
  // Eigen::Matrix<double, 7, 1> expected_vector;
  Eigen::Matrix<double, 3, 1> temp_translation(
      expected_matrix.topRightCorner<3, 1>());
  expected_vector[0] = temp_translation[0];
  expected_vector[1] = temp_translation[1];
  expected_vector[2] = temp_translation[2];
  vector<double> temp;
  Rotationmatrix2Quaternion(expected_matrix.topLeftCorner<3, 3>(), temp);
  expected_vector[3] = temp[0];
  expected_vector[4] = temp[1];
  expected_vector[5] = temp[2];
  expected_vector[6] = temp[3];

  // 发布lidar->vehicle tf
  vehicle_lidar.header.frame_id = "vehicle" + to_string(count);
  vehicle_lidar.child_frame_id = "lidar" + to_string(count);
  vehicle_lidar.header.stamp = now_time;
  vehicle_lidar.transform.translation.x = expected_vector[0];
  vehicle_lidar.transform.translation.y = expected_vector[1];
  vehicle_lidar.transform.translation.z = expected_vector[2];
  vehicle_lidar.transform.rotation.x = expected_vector[3];
  vehicle_lidar.transform.rotation.y = expected_vector[4];
  vehicle_lidar.transform.rotation.z = expected_vector[5];
  vehicle_lidar.transform.rotation.w = expected_vector[6];
  vehicle_lidar_transforms.push_back(vehicle_lidar);
}

void Fusion::Inss2Callback(const geometry_msgs::TransformStamped& inss) {
  // 发布vehicle的tf
  // geometry_msgs::TransformStamped world_vehicle;
  world_vehicle.header.frame_id = "world";
  world_vehicle.child_frame_id = "vehicle" + to_string(count);
  world_vehicle.header.stamp = now_time;

  // 发布vehicle的path
  geometry_msgs::PoseStamped vehicle_pose_stamped;
  vehicle_pose_stamped.header.stamp = now_time;
  vehicle_pose_stamped.header.frame_id = "world";

  // std::cout << "Inss2Callback: " << count << std::endl;

  world_vehicle.transform.translation.x = inss.transform.translation.x;
  world_vehicle.transform.translation.y = inss.transform.translation.y;
  world_vehicle.transform.translation.z = inss.transform.translation.z;

  // std::cout << inss.transform.translation.x << " " << init_x << std::endl;

  world_vehicle.transform.rotation.x = inss.transform.rotation.x;
  world_vehicle.transform.rotation.y = inss.transform.rotation.y;
  world_vehicle.transform.rotation.z = inss.transform.rotation.z;
  world_vehicle.transform.rotation.w = inss.transform.rotation.w;

  world_vehicle_transforms.push_back(world_vehicle);

  vehicle_pose_stamped.pose.position.x = inss.transform.translation.x;
  vehicle_pose_stamped.pose.position.y = inss.transform.translation.y;
  vehicle_pose_stamped.pose.position.z = inss.transform.translation.z;
  vehicle_pose_stamped.pose.orientation.x = inss.transform.rotation.x;
  vehicle_pose_stamped.pose.orientation.y = inss.transform.rotation.y;
  vehicle_pose_stamped.pose.orientation.z = inss.transform.rotation.z;
  vehicle_pose_stamped.pose.orientation.w = inss.transform.rotation.w;
  vehicle_msg.poses.push_back(vehicle_pose_stamped);

  vehicle_pub.publish(vehicle_msg);
}

// 验证标定结果
void Fusion::PubTF(Fusion::OptData* opt_data, ros::NodeHandle* nh) {
  ros::Rate loop_rate(1);
  // 发布过滤后的点云
  filt_pcl_pub = nh->advertise<sensor_msgs::PointCloud2>("/filt_pcl_output", 1);
  // 发布vehicle轨迹
  vehicle_pub = nh->advertise<nav_msgs::Path>("/vehicle_message", 1);
  // 订阅lidar
  lidar_sub = nh->subscribe("/rslidar_points", 1, PointCloud2Callback);
  // 订阅inss
  inss_sub = nh->subscribe("/odom_data", 1, Inss2Callback);
  //   filt_lidar_sub = nh->subscribe("/filt_pcl_output", 1,
  //   NewPointCloud2Callback);

  vehicle_msg.header.stamp = now_time;
  vehicle_msg.header.frame_id = "world";

  tf::TransformBroadcaster broadcaster;

  while (ros::ok()) {
    now_time = ros::Time::now();
    broadcaster.sendTransform(world_vehicle_transforms);
    broadcaster.sendTransform(vehicle_lidar_transforms);
    // for (auto i : output_vector) {
    //   filt_pcl_pub.publish(i);
    // }
    // std::cout << "PubTF: " << count << std::endl;

    count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

}  // namespace lidar_imu
