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
  double oldStartAngle;
  double newStartAngle;
  double oldEndAngle;
  double newEndAngle;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param("start_angle", newStartAngle, -999.9);
  n.param("end_angle", newEndAngle, 999.9);

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
      oldStartAngle = (double)outputRange.startAngle/10000.0 * DEG2RAD - M_PI/2;
      oldEndAngle = (double)outputRange.stopAngle/10000.0 * DEG2RAD - M_PI/2;

      newStartAngle = newStartAngle * DEG2RAD;
      newEndAngle = newEndAngle * DEG2RAD;

      if(newStartAngle > newEndAngle)
      {
        ROS_WARN("Min angle larger than max angle. Flipping min and max angles.");

        double hold = newStartAngle;
        newStartAngle = newEndAngle;
        newEndAngle = hold;
      }

      if(newStartAngle < oldStartAngle)
      {
        scan_msg.angle_min = oldStartAngle;
        newStartAngle = oldStartAngle;
      }
      else
      {
        scan_msg.angle_min = newStartAngle;
      }
      
      if(newEndAngle > oldEndAngle)
      {
        scan_msg.angle_max = oldEndAngle;
        newEndAngle = oldEndAngle;
      }
      else
      {
        scan_msg.angle_max = newEndAngle;
      }

      ROS_DEBUG_STREAM("resolution : " << (double)outputRange.angleResolution/10000.0 << " deg");
      ROS_DEBUG_STREAM("frequency : " << (double)cfg.scaningFrequency/100.0 << " Hz");

      int angle_range = newEndAngle - newStartAngle;
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

      int startVal = (newStartAngle - oldStartAngle) / outputRange.angleResolution;

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

        for (int i = startVal; i < startVal+num_values; i++)
        {
          scan_msg.ranges[i-startVal] = data.dist1[i] * 0.001;
        }

        for (int i = startVal; i < startVal+num_values; i++)
        {
          scan_msg.intensities[i-startVal] = data.rssi1[i];
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
