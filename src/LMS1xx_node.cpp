/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 *  Author: Shravan S Rai  <shravansomashekara.rai@gmail.com>
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "LMS1xx/LMS1xx.h"
#include "LMS1xx/LMS1xx_node.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>

#define DEG2RAD M_PI/180.0

namespace LMS1xx_node
{

LMS1xx_node::LMS1xx_node(rclcpp::NodeOptions options) : Node("LMS1xx_node", options)
{
  host_ = this->declare_parameter("host", "192.168.1.2");
  frame_id_ = this->declare_parameter("frame_id", "laser_link");
  port_ = this->declare_parameter("port", 2111);
  inf_range_ = this->declare_parameter("publish_min_range_as_inf", false);
  laserscan_pub_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);

}

void LMS1xx_node::connect_lidar()
{
  do {
    RCLCPP_INFO(this->get_logger(), "Connecting to Lidar");
    laser_.connect(host_, port_);

    const int MAX_ATTEMPTS = 100;

    if (!laser_.isConnected())
    {
      //Attempt to connect to the lidar a couple of times before giving up
      RCLCPP_INFO(this->get_logger(), "Connecting to Lidar");
      
      while (!laser_.isConnected())
      {
        laser_.connect(host_, port_);

        RCLCPP_WARN(this->get_logger(), "Unable to connect, retrying (%d/%d).", reconnect_timeout_, MAX_ATTEMPTS);

        if (reconnect_timeout_ > MAX_ATTEMPTS)
        {
          RCLCPP_FATAL(this->get_logger(),
                      "Unable to connect to Lidar "
                      "after %d attempts!", MAX_ATTEMPTS);
          return;
        }
        else
        {
          reconnect_timeout_++;
        }

        // timer
        rclcpp::sleep_for(std::chrono::seconds(5));      
      }
    }
    RCLCPP_INFO(this->get_logger(), "Logging in to laser.");
    laser_.login();
    cfg_ = laser_.getScanCfg();
    outputRange_ = laser_.getScanOutputRange();

    RCLCPP_INFO(this->get_logger(), "Connected to laser.");
    construct_scan();
  } while (get_measurements());
}

void LMS1xx_node::construct_scan()
{
  scan_msg_.header.frame_id = frame_id_;
  scan_msg_.range_min = 0.01;
  scan_msg_.range_max = 20.0;
  scan_msg_.scan_time = 100.0 / cfg_.scanningFrequency;
  scan_msg_.angle_increment =
      static_cast<double>(outputRange_.angleResolution) / 10000.0 * DEG2RAD;
  
  scan_msg_.angle_min =
        (static_cast<double>(cfg_.startAngle) / 10000.0) * DEG2RAD - M_PI / 2;
  scan_msg_.angle_max = ((static_cast<double>(cfg_.stopAngle)) / 10000.0) * DEG2RAD - M_PI / 2;

  int angle_range = outputRange_.stopAngle - outputRange_.startAngle;
  int num_values = angle_range/ outputRange_.angleResolution;

  if (angle_range % outputRange_.angleResolution == 0)
  {
    //Include endpoints
    ++num_values;
  }

  scan_msg_.ranges.resize(num_values);
  scan_msg_.intensities.resize(num_values);

  scan_msg_.time_increment = (outputRange_.angleResolution / 10000.0) / 360.0 /
                            (cfg_.scanningFrequency / 100.0);
}

bool LMS1xx_node::get_measurements()
{
  dataCfg_.outputChannel = 1;
  dataCfg_.remission = true;
  dataCfg_.resolution = 1;
  dataCfg_.encoder = 0;
  dataCfg_.position = false;
  dataCfg_.deviceName = false;
  dataCfg_.outputInterval = 1;

  RCLCPP_INFO(this->get_logger(), "Setting scan data configuration.");
  laser_.setScanDataCfg(dataCfg_);

  RCLCPP_INFO(this->get_logger(), "Starting scan measurement.");
  laser_.startMeas();

  RCLCPP_INFO(this->get_logger(), "Waiting for ready status.");
  rclcpp::Time ready_status_timeout =
      this->get_clock()->now() + rclcpp::Duration::from_seconds(5);

  status_t stat = laser_.queryStatus();
  rclcpp::Duration::from_seconds(1);
  if (stat != ready_for_measurement)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Laser not ready. Retrying "
                 "initialization.");
    laser_.disconnect();
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "Starting device.");
  laser_.startDevice();  // Log out to properly re-enable system after config

  RCLCPP_INFO(this->get_logger(), "Commanding continuous measurements.");
  laser_.scanContinous(1);

  while (rclcpp::ok())
  {
    scan_msg_.header.stamp = this->get_clock()->now();
    scanData data;
    if (laser_.getScanData(&data))
    {
      for (int i = 0; i < data.dist_len1; i++)
      {
        float range_data = data.dist1[i] * 0.001;
        if(inf_range_ && scan_msg_.range_min)
        {
          scan_msg_.ranges[i] = std::numeric_limits<float>::infinity(); 
        }
        else
        {
          scan_msg_.ranges[i] = range_data;
        }
        
      }

      for (int i = 0; i < data.rssi_len1; i++)
      {
        scan_msg_.intensities[i] = data.rssi1[i];
      }
      RCLCPP_DEBUG(this->get_logger(), "Publishing the scan data");
      publish_scan();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Laser timed out on delivering scan, "
                   "attempting to reinitialize.");
      return true;
    }
  }
  laser_.scanContinous(0);
  laser_.stopMeas();
  laser_.disconnect();

  return false;
}

void LMS1xx_node::publish_scan()
{
   laserscan_pub_->publish(scan_msg_); 
}

} //namespace LMS1xx_node

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto sick_lms1xx_node = std::make_shared<LMS1xx_node::LMS1xx_node>(options);
  exec.add_node(sick_lms1xx_node);
  sick_lms1xx_node->connect_lidar();
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
