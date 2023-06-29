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

#define DEG2RAD M_PI / 180.0

namespace LMS1xx_node
{

  LMS1xx_node::LMS1xx_node() : Node("LMS1xx_node"),
                               state_(LMS1xxNodeState::CONNECTING),
                               timed_out_(false)
  {
    this->declare_parameter("host", "192.168.131.14");
    this->declare_parameter("frame_id", "laser_link");
    this->declare_parameter("port", 2111);
    this->declare_parameter("publish_min_range_as_inf", false);

    host_ = this->get_parameter("host").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    port_ = this->get_parameter("port").as_int();
    inf_range_ = this->get_parameter("publish_min_range_as_inf").as_bool();

    RCLCPP_INFO(this->get_logger(), "Host %s Port %d", host_.c_str(), port_);

    laserscan_pub_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);

    spin_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() -> void
        {
          this->spin_once();
        });
  }

  void LMS1xx_node::spin_once()
  {
    switch (state_)
    {
    case LMS1xxNodeState::CONNECTING:
      if (connect_lidar())
      {
        reconnect_timeout_ = 0;
        start_measurements();
        state_ = LMS1xxNodeState::MEASURING;
      }
      else if (++reconnect_timeout_ >= MAX_CONNECT_ATTEMPTS)
      {
        RCLCPP_FATAL(this->get_logger(),
                     "Unable to connect to Lidar "
                     "after %d attempts!",
                     MAX_CONNECT_ATTEMPTS);
        state_ = LMS1xxNodeState::ERROR;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unable to connect, retrying (%d/%d).", reconnect_timeout_, MAX_CONNECT_ATTEMPTS);
        state_ = LMS1xxNodeState::TIMEOUT;
      }
      break;

    case LMS1xxNodeState::TIMEOUT:
      if (timeout_timer_ == nullptr || timeout_timer_->is_canceled())
      {
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() -> void
            {
              this->timed_out_ = true;
            });
      }

      if (timed_out_)
      {
        timeout_timer_->cancel();
        timed_out_ = false;
        state_ = LMS1xxNodeState::CONNECTING;
      }
      break;

    case LMS1xxNodeState::MEASURING:
      if (!get_measurements())
      {
        state_ = LMS1xxNodeState::CONNECTING;
      }
      break;

    case LMS1xxNodeState::ERROR:
      RCLCPP_FATAL(this->get_logger(), "Terminating node");
      // Stop laser
      stop_measurements();
      // Cancel timer
      spin_timer_->cancel();
      // Shut down RCLCPP
      rclcpp::shutdown();
      break;
    }
  }

  bool LMS1xx_node::connect_lidar()
  {
    RCLCPP_INFO(this->get_logger(), "Connecting to Lidar");
    laser_.connect(host_, port_);

    if (!laser_.isConnected())
    {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Logging in to laser.");
    laser_.login();
    cfg_ = laser_.getScanCfg();
    outputRange_ = laser_.getScanOutputRange();

    RCLCPP_INFO(this->get_logger(), "Connected to laser.");
    construct_scan();
    return true;
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
    int num_values = angle_range / outputRange_.angleResolution;

    if (angle_range % outputRange_.angleResolution == 0)
    {
      // Include endpoints
      ++num_values;
    }

    scan_msg_.ranges.resize(num_values);
    scan_msg_.intensities.resize(num_values);

    scan_msg_.time_increment = (outputRange_.angleResolution / 10000.0) / 360.0 /
                               (cfg_.scanningFrequency / 100.0);
  }

  void LMS1xx_node::start_measurements()
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

    status_t stat = laser_.queryStatus();

    if (stat != ready_for_measurement)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Laser not ready. Retrying "
                   "initialization.");
      laser_.disconnect();
      state_ = LMS1xxNodeState::CONNECTING;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting device.");
    laser_.startDevice(); // Log out to properly re-enable system after config

    RCLCPP_INFO(this->get_logger(), "Commanding continuous measurements.");
    laser_.scanContinous(1);
  }

  void LMS1xx_node::stop_measurements()
  {
    laser_.scanContinous(0);
    laser_.stopMeas();
    laser_.disconnect();
  }

  bool LMS1xx_node::get_measurements()
  {
    scanData data;
    if (laser_.getScanData(&data))
    {
      publish_scan(data);
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Laser timed out on delivering scan, "
                   "attempting to reinitialize.");
      return false;
    }
  }

  void LMS1xx_node::publish_scan(scanData &data)
  {
    scan_msg_.header.stamp = this->get_clock()->now();

    for (int i = 0; i < data.dist_len1; i++)
    {
      float range_data = data.dist1[i] * 0.001;
      if (inf_range_ && scan_msg_.range_min)
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
    laserscan_pub_->publish(scan_msg_);
  }

} // namespace LMS1xx_node

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto sick_lms1xx_node = std::make_shared<LMS1xx_node::LMS1xx_node>();
  exec.add_node(sick_lms1xx_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
