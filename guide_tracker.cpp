#include "../include/sensors_project/guide_tracker.h"
#include "../include/sensors_project/obstacleDetector.h" // Include the obstacle detector header

GuideTracker::GuideTracker(ros::NodeHandle nh)
    : nh_(nh), obstacle_detected_(false), obstacle_reported_(false), search_reported_(false),
      sweep_complete_(false), start_time_(ros::Time::now()), duration_(0) {
  tracker_sub_ = nh_.subscribe("/visp_auto_tracker/object_position", 1000, &GuideTracker::trackerCallback, this);
  laser_sub_ = nh_.subscribe("/base_scan_raw", 100, &GuideTracker::laserCallBack, this);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  guide_.threshold_distance = 1.0; // You might need to adjust this threshold based on your robot's needs.
  ROS_INFO_STREAM("GuideTracker initialized");
}

GuideTracker::~GuideTracker() {}

void GuideTracker::trackerCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
  // Assuming that the msg provides the position of the QR code in a similar fashion to your example with the ArUco marker.
  // Here we will assume that the x-coordinate corresponds to the forward/backward distance and the y-coordinate corresponds to the left/right distance.
  guide_.pose = msg->pose; // Assuming that the msg has a 'pose' field.

  double distance_to_guide = std::hypot(guide_.pose.position.x, guide_.pose.position.y);

  // Maintain 1m distance between robot and guide
  if (std::abs(distance_to_guide - guide_.threshold_distance) <= 0.1) {
    twistMsg_.linear.x = 0;
    twistMsg_.angular.z = 0;
  } else {
    // Proportional control for both linear and angular velocities
    double linear_error = distance_to_guide - guide_.threshold_distance;
    twistMsg_.linear.x = std::clamp(linear_error, -0.5, 0.5); // Clamp to max/min speeds you define
    twistMsg_.angular.z = std::clamp(-guide_.pose.position.y, -1.0, 1.0); // Assuming negative y requires a positive angular velocity to turn right
  }

  if (!obstacle_detected_) {
    vel_pub_.publish(twistMsg_);
  }

  start_time_ = ros::Time::now();
}

void GuideTracker::laserCallBack(const sensor_msgs::LaserScan::ConstPtr &msg) {
  obstacle_detected_ = obstacleDetector_.isObstacleClose(msg);
  if (obstacle_detected_ && !obstacle_reported_) {
    ROS_INFO("Obstacle detected, stopping the robot.");
    haltMotion();
    obstacle_reported_ = true;
  } else if (!obstacle_detected_ && obstacle_reported_) {
    ROS_INFO("Path clear, obstacle no longer detected.");
    obstacle_reported_ = false;
  }
}

void GuideTracker::haltMotion() {
  twistMsg_.linear.x = 0;
  twistMsg_.angular.z = 0;
  vel_pub_.publish(twistMsg_);
}

void GuideTracker::searchForGuide() {
  // You can use a similar logic to 'GuiderFollow::stop' to implement a search pattern
  // for the QR code when it is not detected.
  // ...
}

void GuideTracker::run() {
  ros::Rate rate(10); // 10 Hz, adjust as appropriate
  while (ros::ok()) {
    ros::spinOnce();
    if (!guide_.detected) {
      searchForGuide();
    }
    rate.sleep();
  }
}

