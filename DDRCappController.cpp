#include "DDRCappController.h"
#include "pose.h"
#include <iostream>

double breach(double dist, double angle, double rmax){
	// return (rmax - dist) / (dist - rmin)
	//return (rmax - std::min(rmax,dist)) / (dist +0.001);
	return 1/dist;
}
DDRCappController::DDRCappController(
    const std::vector<std::pair<double, double>>& path,
    double lookahead_distance,
    double max_linear_speed,
    double max_angular_speed
) : path(path),
    lookahead_distance(lookahead_distance),
    max_linear_speed(max_linear_speed),
    max_angular_speed(max_angular_speed),
    v(0.0),
    w(0.0),
    angle_error(0.0),
    current_index(0),
    ALPHA(0.5),
    BETA(100.0),
    GAMMA(5.0),
    model_weights(361, 0.0),
    rmax(360, 1000.0)
{
    // Initialize model weights
    for (int i = 0; i < 360; ++i) {
        // Sin 2x weights on one half
        if (i < 90 || i > 270) {
        //if (i < 140) {
            model_weights[i] = -std::sin(2.0 * i * M_PI / 180.0);
        } else {
            model_weights[i] = 0.0;
        }
	rmax[i] = std::min(600.0/(std::abs(std::sin(i*M_PI/180)) + 0.001), 1000.0  );
    }
    model_weights[360] = 0.0;
}

void DDRCappController::setPath(const std::vector<std::pair<double, double>>& new_path) {
    path = new_path;
    current_index = 0;
}

void DDRCappController::reset() {
    current_index = 0;
    v = 0.0;
    w = 0.0;
    angle_error = 0.0;
}

std::pair<double, double>* DDRCappController::findLookaheadPoint(const Pose2D& robot_pose) {
    double x = robot_pose.x_;
    double y = robot_pose.y_;
    
    while (current_index < path.size()) {
        double px = path[current_index].first;
        double py = path[current_index].second;
        double dist = std::hypot(px - x, py - y);
        
        if (dist >= lookahead_distance) {
            return &path[current_index];
        }
        current_index++;
    }
    
    return nullptr;
}

double DDRCappController::computeAngle(const Pose2D& robot_pose) {
    double x = robot_pose.x_;
    double y = robot_pose.y_;
    double theta = robot_pose.theta_;
    
    std::pair<double, double>* lookahead_point = findLookaheadPoint(Pose2D{x, y, theta});
    
    if (lookahead_point == nullptr) {
        return 0.0; // Stop if path is complete
    }
    
    double lx = lookahead_point->first;
    double ly = lookahead_point->second;
    
    // Transform to robot frame
    double dx = lx - x;
    double dy = ly - y;
    double transformed_x = std::cos(theta) * dx + std::sin(theta) * dy;
    double transformed_y = -std::sin(theta) * dx + std::cos(theta) * dy;
    
    if (transformed_x <= 0) {
        // Don't drive backwards
        return (transformed_y >= 0) ? 1.57 : -1.57;
    }
    
    return std::atan2(transformed_y, transformed_x);
}

std::pair<double, bool> DDRCappController::computeCAControl(const Scan& scan_array) {
    bool warn_flag = false;
    
    // scan_array is Nx3 matrix: [index, angle, distance]
    // Columns: 0=index, 1=angle, 2=distance
    int num_points = scan_array.rows();
    
    double score_value = 0.0;
    
    for (int i = 0; i < num_points; ++i) {
        double angle = scan_array(i, 0);
        double distance = scan_array(i, 1)*1000.0; // Convert to mm
        
        // Compute index
        int idx = static_cast<int>(std::floor(angle*180/M_PI));
        idx = std::min(359, idx);
        
        // Accumulate score
	if (distance > 0.01 && distance < 1000.0)
	
        score_value += breach(distance,angle,rmax[idx]) * model_weights[idx];
        
        // Check for proximity warning
        if (distance < 100.0) {
            warn_flag = true;
        }
    }
    
    score_value -= model_weights[360];
    
    double w_ca = -1.0*GAMMA * score_value;
    
    return std::make_pair(w_ca, warn_flag);
}

std::pair<int, int> DDRCappController::computeControl(
    const Pose2D& robot_pose_est,
    const Scan& scan_array
) {
    // Compute angle error using Pure Pursuit
    if (!path.empty()) {
        angle_error = computeAngle(robot_pose_est);
    } else {
        angle_error = 0.0;
    }
    std::cout << "angle_error: " << angle_error << "\n";

    // Compute collision avoidance control
    auto ca_result = computeCAControl(scan_array);
    double w_ca_raw = ca_result.first;
    bool warn_flag = ca_result.second;
    
    // Combine controllers
    double w_combined = 200.0 * w_ca_raw + 150.0 * angle_error * std::exp(-w_ca_raw * w_ca_raw);
    std::cout << "raw: "<< w_ca_raw << " CA term: " << 200.0 * w_ca_raw  << " Track term: " << 150.0 * angle_error * std::exp(-w_ca_raw * w_ca_raw) << "\n";
    
    // Clamp angular velocity
    if (w_combined > max_angular_speed) {
        w_combined = max_angular_speed;
    }
    if (w_combined < -max_angular_speed) {
        w_combined = -max_angular_speed;
    }
    
    // Low-pass filter
    w_combined = 0.9 * w_combined + 0.1 * w;
    w = w_combined;
    
    // Compute forward velocity
    double v_computed = max_linear_speed * std::exp(-ALPHA * std::abs(w_ca_raw));
    
    // Safety check
    if (warn_flag) {
        v_computed = 0.0;
    }
    
    v = v_computed;
    
    return std::make_pair(static_cast<int>(v), static_cast<int>(w));
}
