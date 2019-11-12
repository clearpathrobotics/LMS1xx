#include <string>
#include <thread>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <LMS1xx/LMS1xx.h>
#include <sensor_msgs/LaserScan.h>

#include <pluginlib/class_list_macros.h>

#define DEG2RAD M_PI/180.0

namespace lms1xx {

class LMS1xxNodelet : public nodelet::Nodelet {

public:
  virtual ~LMS1xxNodelet();

protected:
  void onInit() override;
  void loop();

  std::thread thread;

  LMS1xx laser;
  scanCfg cfg;
  scanOutputRange outputRange;
  scanDataCfg dataCfg;
  ros::Publisher scan_pub, scan2_pub;

  // parameters
  std::string host;
  std::string frame_id;
  bool inf_range;
  bool publish_2nd_pulse;
  int port;
  float range_min, range_max;

  ros::NodeHandle nh, n;

  volatile bool isShuttingDown = false;

  bool ok() const {
    return !isShuttingDown && ros::ok();
  }
};


void LMS1xxNodelet::onInit() {
  nh = getNodeHandle();
  n = getPrivateNodeHandle();

  n.param<std::string>("host", host, "192.168.1.2");
  n.param<std::string>("frame_id", frame_id, "laser");
  n.param<bool>("publish_min_range_as_inf", inf_range, false);
  n.param<bool>("publish_2nd_pulse", publish_2nd_pulse, false);
  n.param<float>("range_min", range_min, 0.01);
  n.param<float>("range_min", range_max, 20.0);
  n.param<int>("port", port, 2111);

  if (publish_2nd_pulse)
    NODELET_DEBUG("Reading and publishing also second laser reflections.");

  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  if (publish_2nd_pulse)
    scan2_pub = nh.advertise<sensor_msgs::LaserScan>("scan_2nd_pulse", 1);

  thread = std::thread(&LMS1xxNodelet::loop, this);
}

void LMS1xxNodelet::loop() {
  while (this->ok()) {
    NODELET_INFO_STREAM("Connecting to laser at " << host);
    laser.connect(host, port);
    if (!laser.isConnected()) {
      NODELET_WARN("Unable to connect, retrying.");
      ros::WallDuration(1).sleep();
      continue;
    }

    NODELET_DEBUG("Logging in to laser.");
    laser.login();
    cfg = laser.getScanCfg();
    outputRange = laser.getScanOutputRange();

    if (cfg.scaningFrequency != 5000) {
      laser.disconnect();
      NODELET_WARN("Unable to get laser output range. Retrying.");
      ros::WallDuration(1).sleep();
      continue;
    }

    NODELET_INFO("Connected to laser.");

    NODELET_DEBUG("Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
                  cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
                  cfg.stopAngle);
    NODELET_DEBUG(
        "Laser output range:angleResolution %d, startAngle %d, stopAngle %d",
        outputRange.angleResolution, outputRange.startAngle,
        outputRange.stopAngle);

    sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
    sensor_msgs::LaserScanPtr scan2_msg(new sensor_msgs::LaserScan());

    scan_msg->header.frame_id = scan2_msg->header.frame_id = frame_id;
    scan_msg->range_min = scan2_msg->range_min = range_min;
    scan_msg->range_max = scan2_msg->range_max = range_max;
    scan_msg->scan_time = scan2_msg->scan_time = 100.0 / cfg.scaningFrequency;
    scan_msg->angle_increment = scan2_msg->angle_increment =
        static_cast<double>(outputRange.angleResolution / 10000.0 * DEG2RAD);
    scan_msg->angle_min = scan2_msg->angle_min = static_cast<double>(
        outputRange.startAngle / 10000.0 * DEG2RAD - M_PI / 2);
    scan_msg->angle_max = scan2_msg->angle_max = static_cast<double>(
        outputRange.stopAngle / 10000.0 * DEG2RAD - M_PI / 2);

    NODELET_DEBUG_STREAM("Device resolution is "
                         << (double)outputRange.angleResolution / 10000.0
                         << " degrees.");
    NODELET_DEBUG_STREAM("Device frequency is "
                         << (double)cfg.scaningFrequency / 100.0 << " Hz");

    int angle_range = outputRange.stopAngle - outputRange.startAngle;
    int num_values = angle_range / outputRange.angleResolution;
    if (angle_range % outputRange.angleResolution == 0) {
      // Include endpoint
      ++num_values;
    }
    scan_msg->ranges.resize(num_values);
    scan_msg->intensities.resize(num_values);
    if (publish_2nd_pulse) {
      scan2_msg->ranges.resize(num_values);
      scan2_msg->intensities.resize(num_values);
    }

    scan_msg->time_increment = scan2_msg->time_increment =
        (outputRange.angleResolution / 10000.0) / 360.0 /
        (cfg.scaningFrequency / 100.0);

    NODELET_DEBUG_STREAM("Time increment is "
                         << static_cast<int>(scan_msg->time_increment * 1000000)
                         << " microseconds");

    dataCfg.outputChannel = publish_2nd_pulse ? 3 : 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    NODELET_DEBUG("Setting scan data configuration.");
    laser.setScanDataCfg(dataCfg);

    NODELET_DEBUG("Starting measurements.");
    laser.startMeas();

    NODELET_DEBUG("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    // while(1)
    // {
    status_t stat = laser.queryStatus();
    ros::WallDuration(1.0).sleep();
    if (stat != ready_for_measurement) {
      NODELET_WARN("Laser not ready. Retrying initialization.");
      laser.disconnect();
      ros::WallDuration(1).sleep();
      continue;
    }
    /*if (stat == ready_for_measurement)
    {
      NODELET_DEBUG("Ready status achieved.");
      break;
    }

      if (ros::Time::now() > ready_status_timeout)
      {
        NODELET_WARN("Timed out waiting for ready status. Trying again.");
        laser.disconnect();
        continue;
      }

      if (!ros::ok())
      {
        laser.disconnect();
        return 1;
      }
    }*/

    NODELET_DEBUG("Starting device.");
    laser.startDevice(); // Log out to properly re-enable system after config

    NODELET_DEBUG("Commanding continuous measurements.");
    laser.scanContinous(1);

    while (this->ok()) {
      ros::Time start = ros::Time::now();

      scan_msg->header.stamp = scan2_msg->header.stamp = start;
      ++scan_msg->header.seq;
      ++scan2_msg->header.seq;

      scanData data;
      NODELET_DEBUG("Reading scan data.");
      if (laser.getScanData(&data)) {
        for (int i = 0; i < data.dist_len1; i++) {
          float range_data = data.dist1[i] * 0.001f;

          if (inf_range && range_data < scan_msg->range_min) {
            scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
          } else {
            scan_msg->ranges[i] = range_data;
          }
        }

        for (int i = 0; i < data.rssi_len1; i++) {
          scan_msg->intensities[i] = data.rssi1[i];
        }

        NODELET_DEBUG("Publishing scan data.");
        scan_pub.publish(scan_msg);

        if (publish_2nd_pulse) {
          for (int i = 0; i < data.dist_len2; i++) {
            float range_data = data.dist2[i] * 0.001f;

            if (inf_range && range_data < scan2_msg->range_min) {
              scan2_msg->ranges[i] = std::numeric_limits<float>::infinity();
            } else {
              scan2_msg->ranges[i] = range_data;
            }
          }

          for (int i = 0; i < data.rssi_len2; i++) {
            scan2_msg->intensities[i] = data.rssi2[i];
          }

          NODELET_DEBUG("Publishing second pulse scan data.");
          scan2_pub.publish(scan2_msg);
        }
      } else {
        NODELET_ERROR(
            "Laser timed out on delivering scan, attempting to reinitialize.");
        break;
      }
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }
}

LMS1xxNodelet::~LMS1xxNodelet() {
  this->isShuttingDown;
  this->thread.join();
}

}

PLUGINLIB_EXPORT_CLASS(lms1xx::LMS1xxNodelet, nodelet::Nodelet)