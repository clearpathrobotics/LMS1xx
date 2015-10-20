#include <csignal>
#include <cstdio>
#include <LMS1xx/LMS1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser;
  scanCfg scan_config;
  scanOutputRange output_config;

  // parameters
  std::string host;
  std::string frame_id;

  const std::string& node_name = "lms1xx_configuration";
  ros::init(argc, argv, node_name);
  ros::NodeHandle n("~");

  n.param<std::string>("host", host, "192.168.1.2");
  int time = 10;
  n.param<int>("time_out", time, time);

  ros::Duration time_out(time);
  ros::Time starting_time = ros::Time::now();

  // reading current SICK configuration
  while(!laser.isConnected()) {
    ros::Duration d(ros::Time::now()-starting_time);
    if (d>time_out)
    {
      ROS_ERROR_STREAM_NAMED(node_name, "Time out while trying to connect to laser " << host);
      exit(-1);
    }

    laser.connect(host);
    if(laser.isConnected())
    {
      laser.scanContinous(0);
      laser.stopMeas();
      laser.login();
      scan_config = laser.getScanCfg();
    }
    ROS_INFO_STREAM_NAMED(node_name, "Waiting for laser to connect!");
  }

  // read new configuration from param server
  n.param<int>("scaning_frequency",  scan_config.scaningFrequency, scan_config.scaningFrequency);
  n.param<int>("angle_resolution",   scan_config.angleResolution,  scan_config.angleResolution);
  n.param<int>("start_angle",        scan_config.startAngle,       scan_config.startAngle);
  n.param<int>("stop_angle",         scan_config.stopAngle,        scan_config.stopAngle);
  bool make_persistent = false;
  n.param<bool>("make_persistent",   make_persistent,              make_persistent);

  output_config.angleResolution = scan_config.angleResolution;
  output_config.startAngle      = scan_config.startAngle;
  output_config.stopAngle       = scan_config.stopAngle;

  laser.setScanCfg(scan_config);
  laser.setOutputRange(output_config);
  if (make_persistent)
  {
    laser.saveConfig(); // make permanently
  }

  scan_config = laser.getScanCfg();
  output_config = laser.getScanOutputRange();

  ROS_INFO("New laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
                scan_config.scaningFrequency, scan_config.angleResolution, scan_config.startAngle, scan_config.stopAngle);
  ROS_INFO("New laser output range:angleResolution %d, startAngle %d, stopAngle %d",
                output_config.angleResolution, output_config.startAngle, output_config.stopAngle);

  ROS_INFO_STREAM_NAMED(node_name, "Shutting down laser " << host);
  laser.disconnect();

  return 0;
}
