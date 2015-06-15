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
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;
  scanData data;
  // published data
  sensor_msgs::LaserScan scan_msg;

  // parameters
  std::string host;
  std::string frame_id;

  // limiting data
  double new_start_angle_rad;
  double new_stop_angle_rad;
  double old_start_angle;
  double new_start_angle;
  double old_stop_angle;
  double new_stop_angle;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param("start_angle", new_start_angle, -999.9);
  n.param("stop_angle", new_stop_angle, 999.9);

  while(ros::ok())
  {
    ROS_INFO("Connecting to laser at : %s", host.c_str());

    // initialize hardware
    do {
      laser.connect(host);

      if (laser.isConnected())
      {	
	laser.login();
	cfg = laser.getScanCfg();
	outputRange = laser.getScanOutputRange();
      }

      //check if laser is fully initialized, else reconnect
      //assuming fully initialized => scaningFrequency=5000
      if (cfg.scaningFrequency != 5000) {
	laser.disconnect();
	ROS_INFO("Waiting for laser to initialize...");
      }

    } while (!laser.isConnected() || cfg.scaningFrequency != 5000);

    if (laser.isConnected()) {
      ROS_INFO("Connected to laser.");

      ROS_DEBUG("Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
                cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle, cfg.stopAngle);
      ROS_DEBUG("Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
                outputRange.angleResolution, outputRange.startAngle, outputRange.stopAngle);

      scan_msg.header.frame_id = frame_id;

      scan_msg.range_min = 0.01;
      scan_msg.range_max = 20.0;

      scan_msg.scan_time = 100.0/cfg.scaningFrequency;

      scan_msg.angle_increment = (double)outputRange.angleResolution/10000.0 * DEG2RAD;
      old_start_angle = (double)outputRange.startAngle/10000.0 * DEG2RAD - M_PI/2;
      old_stop_angle = (double)outputRange.stopAngle/10000.0 * DEG2RAD - M_PI/2;

      new_start_angle_rad = new_start_angle * DEG2RAD;
      new_stop_angle_rad = new_stop_angle * DEG2RAD;
      
      if(new_start_angle_rad > new_stop_angle_rad)
      {
        ROS_WARN("Min angle larger than max angle. Flipping min and max angles.");

        double hold = new_start_angle_rad;
        new_start_angle_rad = new_stop_angle_rad;
        new_stop_angle_rad = hold;
        
        hold = new_start_angle;
        new_start_angle = new_stop_angle;
        new_stop_angle = hold;
      }

      if(new_start_angle_rad < old_start_angle)
      {
        scan_msg.angle_min = old_start_angle;
        new_start_angle_rad = old_start_angle;
        new_start_angle = outputRange.startAngle;
      }
      else
      {
        scan_msg.angle_min = new_start_angle_rad;
        new_start_angle = (new_start_angle + 90) * 10000;
      }
      
      if(new_stop_angle_rad > old_stop_angle)
      {
        scan_msg.angle_max = old_stop_angle;
        new_stop_angle_rad = old_stop_angle;
        new_stop_angle = outputRange.stopAngle;
      }
      else
      {
        scan_msg.angle_max = new_stop_angle_rad;
        new_stop_angle = (new_stop_angle + 90) * 10000;
      }
      
      ROS_DEBUG_STREAM("resolution : " << (double)outputRange.angleResolution/10000.0 << " deg");
      ROS_DEBUG_STREAM("frequency : " << (double)cfg.scaningFrequency/100.0 << " Hz");

      int angle_range = new_stop_angle - new_start_angle;
      int num_values = angle_range / outputRange.angleResolution;
      if (angle_range % outputRange.angleResolution == 0) {
          // Include endpoint
          ++num_values;
      }   

      scan_msg.time_increment =
          (outputRange.angleResolution / 10000.0)
          / 360.0
          / (cfg.scaningFrequency / 100.0);

      ROS_DEBUG_STREAM("time increment : " << (double)scan_msg.time_increment << " seconds");

      scan_msg.ranges.resize(num_values);
      scan_msg.intensities.resize(num_values);

      int start_val = (new_start_angle - (double)outputRange.startAngle) / outputRange.angleResolution;

      dataCfg.outputChannel = 1;
      dataCfg.remission = true;
      dataCfg.resolution = 1;
      dataCfg.encoder = 0;
      dataCfg.position = false;
      dataCfg.deviceName = false;
      dataCfg.outputInterval = 1;

      laser.setScanDataCfg(dataCfg);

      laser.startMeas();

      status_t stat;
      do // wait for ready status
      {
        stat = laser.queryStatus();
        ros::Duration(1.0).sleep();
      }
      while (stat != ready_for_measurement);

      laser.startDevice(); // Log out to properly re-enable system after config

      laser.scanContinous(1);

      while (ros::ok())
      {
        ros::Time start = ros::Time::now();

        scan_msg.header.stamp = start;
        ++scan_msg.header.seq;

        laser.getData(data);

        for (int i = start_val; i < start_val+num_values; i++)
        {
          scan_msg.ranges[i-start_val] = data.dist1[i] * 0.001;
        }

        for (int i = start_val; i < start_val+num_values; i++)
        {
          scan_msg.intensities[i-start_val] = data.rssi1[i];
        }

        scan_pub.publish(scan_msg);

        ros::spinOnce();
      }

      laser.scanContinous(0);
      laser.stopMeas();
      laser.disconnect();
    }
    else
    {
      ROS_ERROR("Connection to LMS1xx device failed, retrying in 1 second.");
      ros::Duration(1.0).sleep();
    }
  }

  return 0;
}
