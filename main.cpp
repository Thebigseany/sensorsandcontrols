#include "ros/ros.h"
#include "../include/sensors_project/guide_tracker.h"
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guide_tracker");
    ros::NodeHandle nh;

    GuideTracker robot(nh); // GuideTracker instance is created on the stack.

    std::thread tracking_thread(&GuideTracker::run, &robot); // Thread for running the main loop.

    ros::spin(); // Handles callbacks.

    // Clean up and shutdown.
    ros::shutdown();
    tracking_thread.join(); // Ensure the tracking thread has finished before exiting the program.

    return 0;
}