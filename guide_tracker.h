#ifndef GUIDE_TRACKER_H
#define GUIDE_TRACKER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
// Correct this include to match the actual message type you are working with.
// Assuming LaserScan here.
#include "sensor_msgs/LaserScan.h"
#include "obstacleDetector.h" // Corrected path assuming it's in the same directory.
#include <cmath>
#include <algorithm>

class GuideTracker {
public:
  GuideTracker(ros::NodeHandle nh);
  ~GuideTracker();

  void run(); // Main loop for running the tracker

private:
  // Callback functions
  void trackerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void laserCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);

  // Action methods
  void haltMotion();
  void searchForGuide(); // Define how to search for the guide


  // NodeHandle for ROS communication
  ros::NodeHandle nh_;

  // ROS communication members
  ros::Subscriber tracker_sub_;
  ros::Subscriber laser_sub_;
  ros::Publisher vel_pub_;
  geometry_msgs::Twist twistMsg_;

  // Other member variables
  ObstacleDetector obstacleDetector_; // Assuming LaserDetection is a class
  bool obstacle_detected_;
  bool obstacle_reported_;
  bool search_reported_;
  bool sweep_complete_;
  ros::Time start_time_;
  ros::Duration duration_;

  // Nested struct for Guide-related data
  struct Guide {
    geometry_msgs::Pose pose;
    double threshold_distance; // Distance to maintain from the guide
    double shortest_dist; // Shortest distance detected to the guide (if needed)
    bool detected; // Flag to determine if the guide is currently detected
    bool reached; // Flag to determine if the guide has been reached (if needed)

    Guide() : threshold_distance(1.0), shortest_dist(std::numeric_limits<double>::infinity()),
              detected(false), reached(false) {}
  };

  Guide guide_; // Instance of Guide struct to manage guide data
};
#endif // GUIDE_TRACKER_H