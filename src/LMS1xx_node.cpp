/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
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

#include <csignal>
#include <cstdio>
#include <lms1xx/lms1xx.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD M_PI/180.0

using namespace lms1xx;

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scan_configuration cfg;
  scan_output_range outputRange;
  scan_data_configuration dataCfg;
  sensor_msgs::LaserScan scan_msg;

  // parameters
  std::string host;
  std::string frame_id;
  std::string port;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<std::string>("port", port, "2111");

  while (ros::ok())
  {
    ROS_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host, port);
    if (!laser.connected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_DEBUG("Logging in to laser.");
    laser.login();
    cfg = laser.get_configuration();
    outputRange = laser.get_scan_output_range();

    if (cfg.scaning_frequency != 5000)
    {
      laser.disconnect();
      ROS_WARN("Unable to get laser output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_INFO("Connected to laser.");

    ROS_DEBUG("Laser configuration: scaning_frequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scaning_frequency, cfg.angle_resolution, cfg.start_angle, cfg.stop_angle);
    ROS_DEBUG("Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
              outputRange.angle_resolution, outputRange.start_angle, outputRange.stop_angle);

    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 100.0 / cfg.scaning_frequency;
    scan_msg.angle_increment = (double)outputRange.angle_resolution / 10000.0 * DEG2RAD;
    scan_msg.angle_min = (double)outputRange.start_angle / 10000.0 * DEG2RAD - M_PI / 2;
    scan_msg.angle_max = (double)outputRange.stop_angle / 10000.0 * DEG2RAD - M_PI / 2;

    ROS_DEBUG_STREAM("Device resolution is " << (double)outputRange.angle_resolution / 10000.0 << " degrees.");
    ROS_DEBUG_STREAM("Device frequency is " << (double)cfg.scaning_frequency / 100.0 << " Hz");

    int angle_range = outputRange.stop_angle - outputRange.start_angle;
    int num_values = angle_range / outputRange.angle_resolution;
    if (angle_range % outputRange.angle_resolution == 0)
    {
      // Include endpoint
      ++num_values;
    }
    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    scan_msg.time_increment =
      (outputRange.angle_resolution / 10000.0)
      / 360.0
      / (cfg.scaning_frequency / 100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

    dataCfg.output_channel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.device_name = false;
    dataCfg.output_interval = 1;

    ROS_DEBUG("Setting scan data configuration.");
    laser.set_scan_data_configuration(dataCfg);

    ROS_DEBUG("Starting measurements.");
    laser.start_measurements();

    ROS_DEBUG("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    //while(1)
    //{
    device_status stat = laser.status();
    ros::Duration(1.0).sleep();
    if (stat != device_status::ready_for_measurement)
    {
      ROS_WARN("Laser not ready. Retrying initialization.");
      laser.disconnect();
      ros::Duration(1).sleep();
      continue;
    }
    /*if (stat == ready_for_measurement)
    {
      ROS_DEBUG("Ready status achieved.");
      break;
    }

      if (ros::Time::now() > ready_status_timeout)
      {
        ROS_WARN("Timed out waiting for ready status. Trying again.");
        laser.disconnect();
        continue;
      }

      if (!ros::ok())
      {
        laser.disconnect();
        return 1;
      }
    }*/

    ROS_DEBUG("Starting device.");
    laser.start_device(); // Log out to properly re-enable system after config

    ROS_DEBUG("Commanding continuous measurements.");
    laser.scan_continous(true);

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      scan_data data;
      ROS_DEBUG("Reading scan data.");
      data = laser.get_data();
      for (int i = 0; i < data.dist_len1; i++)
      {
        scan_msg.ranges[i] = data.dist1[i] * 0.001;
      }

      for (int i = 0; i < data.rssi_len1; i++)
      {
        scan_msg.intensities[i] = data.rssi1[i];
      }

      ROS_DEBUG("Publishing scan data.");
      scan_pub.publish(scan_msg);

      ros::spinOnce();
    }

    laser.scan_continous(false);
    laser.stop_measurements();
    laser.disconnect();
  }

  return 0;
}
