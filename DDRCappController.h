#ifndef DDR_CAPP_CONTROLLER_H
#define DDR_CAPP_CONTROLLER_H

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "pose.h"

// Scan data in polar form: each row is [index, angle, distance]
// Using Eigen::MatrixXd for compatibility with your existing code
using Scan = Eigen::MatrixXd;

class DDRCappController {
public:
    DDRCappController(
        const std::vector<std::pair<double, double>>& path,
        double lookahead_distance = 0.5,
        double max_linear_speed = 150.0,
        double max_angular_speed = 200.0
    );
    
    // Update the path (useful when replanning)
    void setPath(const std::vector<std::pair<double, double>>& path);
    
    // Main control function
    std::pair<int, int> computeControl(const Pose2D& robot_pose_est, const Scan& scan_array);
    
    // Reset the controller state
    void reset();
    
private:
    // Pure Pursuit helpers
    std::pair<double, double>* findLookaheadPoint(const Pose2D& robot_pose);
    double computeAngle(const Pose2D& robot_pose);
    
    // Collision avoidance helper
    std::pair<double, bool> computeCAControl(const Scan& scan_array);
    
    // Member variables
    std::vector<double> model_weights;
    std::vector<double> rmax;
    double ALPHA;
    double BETA;
    double GAMMA;
    double v;
    double w;
    std::vector<std::pair<double, double>> path;
    double angle_error;
    double lookahead_distance;
    double max_linear_speed;
    double max_angular_speed;
    size_t current_index;
};

#endif // DDR_CAPP_CONTROLLER_H
