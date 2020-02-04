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
#include <LMS1xx/LMS1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <limits>
#include <string>

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;
  sensor_msgs::LaserScan scan_msg;

  // parameters
  double angle_max;
  double angle_min;
  std::string host;
  std::string frame_id;
  bool inf_range;
  int port;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<double>("angle_max", angle_max, M_PI);
  n.param<double>("angle_min", angle_min, -M_PI);
  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<bool>("publish_min_range_as_inf", inf_range, false);
  n.param<int>("port", port, 2111);

  if (angle_min > angle_max)
  {
    ROS_FATAL("Minimum angle %f exceeds maximum angle %f", angle_min, angle_max);
    ros::shutdown();
    return 1;
  }

  while (ros::ok())
  {
    ROS_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host, port);
    if (!laser.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_DEBUG("Logging in to laser.");
    laser.login();
    cfg = laser.getScanCfg();
    outputRange = laser.getScanOutputRange();

    if (cfg.scaningFrequency != 5000)
    {
      laser.disconnect();
      ROS_WARN("Unable to get laser output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    ROS_INFO("Connected to laser.");

    ROS_DEBUG("Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle, cfg.stopAngle);
    ROS_DEBUG("Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
              outputRange.angleResolution, outputRange.startAngle, outputRange.stopAngle);

    scan_msg.header.frame_id = frame_id;
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 100.0 / cfg.scaningFrequency;
    scan_msg.angle_increment = static_cast<double>(outputRange.angleResolution / 10000.0 * DEG2RAD);

    const double output_range_start_angle = static_cast<double>(outputRange.startAngle) / 10000.0 * DEG2RAD - M_PI / 2.0;
    const double output_range_stop_angle = static_cast<double>(outputRange.stopAngle) / 10000.0 * DEG2RAD - M_PI / 2.0;
    angle_max = std::min(output_range_stop_angle, angle_max);
    angle_min = std::max(output_range_start_angle, angle_min);

    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;

    ROS_DEBUG_STREAM("Device resolution is " << (double)outputRange.angleResolution / 10000.0 << " degrees.");
    ROS_DEBUG_STREAM("Device frequency is " << (double)cfg.scaningFrequency / 100.0 << " Hz");

    const double angle_resolution = static_cast<double>(outputRange.angleResolution) / 10000.0 * DEG2RAD; 
    const double angle_range = angle_max - angle_min;
    int num_values = round(angle_range / angle_resolution) + 1;

    // Determine start index for incoming data
    const int start_idx = std::ceil((angle_min - output_range_start_angle) / angle_resolution);

    ROS_DEBUG_STREAM("Number of ranges is " << num_values);
    ROS_DEBUG_STREAM("Range start index is " << start_idx);

    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    scan_msg.time_increment =
      (outputRange.angleResolution / 10000.0)
      / 360.0
      / (cfg.scaningFrequency / 100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan_msg.time_increment * 1000000) << " microseconds");

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    ROS_DEBUG("Setting scan data configuration.");
    laser.setScanDataCfg(dataCfg);

    ROS_DEBUG("Starting measurements.");
    laser.startMeas();

    ROS_DEBUG("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    // while(1)
    // {
    status_t stat = laser.queryStatus();
    ros::Duration(1.0).sleep();
    if (stat != ready_for_measurement)
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
    laser.startDevice();  // Log out to properly re-enable system after config

    ROS_DEBUG("Commanding continuous measurements.");
    laser.scanContinous(1);

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      scanData data;
      ROS_DEBUG("Reading scan data.");
      if (laser.getScanData(&data))
      {
        ROS_ASSERT_MSG(data.dist_len1 == data.rssi_len1,
                       "Number of ranges (%d) does not match number of intensities (%d)",
                       data.dist_len1,
                       data.rssi_len1);

        for (int i = 0; i < num_values; i++)
        {
          const float range_data = data.dist1[i + start_idx] * 0.001;

          if (inf_range && range_data < scan_msg.range_min)
          {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
          }
          else
          {
            scan_msg.ranges[i] = range_data;
          }


          scan_msg.intensities[i] = data.rssi1[i + start_idx];
        }

        ROS_DEBUG("Publishing scan data.");
        scan_pub.publish(scan_msg);
      }
      else
      {
        ROS_ERROR("Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      ros::spinOnce();
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }

  return 0;
}
