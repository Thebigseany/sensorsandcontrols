#include "../include/sensors_project/obstacleDetector.h" // Adjust the path as necessary

constexpr int VIEWING_ANGLE = 140; // Angle to scan for obstacles
constexpr double SAFE_DISTANCE = 0.35;     // Safe distance before considering an obstacle too close
constexpr double ROBOT_RADIUS = 0.257;     // Robot's radius for additional safety buffer

ObstacleDetector::ObstacleDetector() {}

bool ObstacleDetector::isObstacleClose(const sensor_msgs::LaserScan::ConstPtr& scan) {
    double min_reading = scan->range_max;
    int index_start = (scan->ranges.size() / 2) - (VIEWING_ANGLE / 2 * scan->ranges.size() / (scan->angle_max - scan->angle_min));
    int index_end = (scan->ranges.size() / 2) + (VIEWING_ANGLE / 2 * scan->ranges.size() / (scan->angle_max - scan->angle_min));

    // Ensure indices are within bounds
    index_start = std::max(0, index_start);
    index_end = std::min(static_cast<int>(scan->ranges.size()) - 1, index_end);

    for (int i = index_start; i <= index_end; i++) {
        if (scan->ranges[i] < min_reading) {
            min_reading = scan->ranges[i];
        }
    }
    return min_reading <= SAFE_DISTANCE + ROBOT_RADIUS;
}

double ObstacleDetector::nearestObstacleDistance(const sensor_msgs::LaserScan::ConstPtr& scan) {
    double nearest_distance = std::numeric_limits<double>::infinity();
    int index_start = (scan->ranges.size() / 2) - (VIEWING_ANGLE / 2 * scan->ranges.size() / (scan->angle_max - scan->angle_min));
    int index_end = (scan->ranges.size() / 2) + (VIEWING_ANGLE / 2 * scan->ranges.size() / (scan->angle_max - scan->angle_min));

    // Ensure indices are within bounds
    index_start = std::max(0, index_start);
    index_end = std::min(static_cast<int>(scan->ranges.size()) - 1, index_end);

    for (int i = index_start; i <= index_end; i++) {
        if (scan->ranges[i] < nearest_distance) {
            nearest_distance = scan->ranges[i];
        }
    }
    return nearest_distance;

}