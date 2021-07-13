#include <ros/ros.h>

#include <algorithm>

#include "fusion.h"
#include "loader.h"

using namespace lidar_imu;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_imu");

  ros::NodeHandle nh, nh_private("~");

  Loader loader(Loader::getConfig(&nh_private));

  std::cout << "start" << std::endl;

  Fusion fusioner(Fusion::getConfig(&nh_private));
  fusioner.PubTF(&nh_private);

  return 0;
}
