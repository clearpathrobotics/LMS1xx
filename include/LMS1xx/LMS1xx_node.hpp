#pragma once

#include "LMS1xx/LMS1xx.h"

#include <chrono>
#include <memory>
#include <string>

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace LMS1xx_node
{
class LMS1xx_node : public rclcpp::Node
{
  public:
  explicit LMS1xx_node(rclcpp::NodeOptions options);

  /**
    * @brief connect to the Sick LMS1xx lidar
    */
  void connect_lidar();

  private:
  // laser data
  LMS1xx laser_;
  scanCfg cfg_;
  scanOutputRange outputRange_;
  scanDataCfg dataCfg_;

  // parameters
  std::string host_;
  std::string frame_id_;
  int port_;
  bool inf_range_;
  int reconnect_timeout_{0};
  sensor_msgs::msg::LaserScan scan_msg_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  /**
    * @brief construct a scan message based on the
    * configuration of the sick lidar
    * @param scan sensor_msgs::msg::laserscan
    */
  void construct_scan();

  /**
    * @brief get measurements from the lidar after it has 
    * been setup properly
    * 
    * returns: if it should reconnect
    */
  bool get_measurements();

  /**
    * @brief publishes scan messages
    */
  void publish_scan();

  /**
    * @brief publishes cloud messages
    */
  void publish_cloud();
};

}  // namespace LMS1xx_node