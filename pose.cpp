#include "pose.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

// Constants
const double TOLERANCE = 1e-9;
const double PI = M_PI;

// Constructors
Pose2D::Pose2D() : x_(0.0), y_(0.0), theta_(0.0) {}

Pose2D::Pose2D(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {}

Pose2D::Pose2D(const Pose2D& other) : x_(other.x_), y_(other.y_), theta_(other.theta_) {}

// Assignment operator
Pose2D& Pose2D::operator=(const Pose2D& other) {
    if (this != &other) {
        x_ = other.x_;
        y_ = other.y_;
        theta_ = other.theta_;
    }
    return *this;
}

// Array-style access operator
double Pose2D::operator[](int index) const {
    switch (index) {
        case 0: return x_;
        case 1: return y_;
        case 2: return theta_;
        default: 
            throw std::out_of_range("Index must be 0, 1, or 2");
    }
}

// Equality operator
bool Pose2D::operator==(const Pose2D& other) const {
    return (std::abs(x_ - other.x_) < TOLERANCE &&
            std::abs(y_ - other.y_) < TOLERANCE &&
            std::abs(theta_ - other.theta_) < TOLERANCE);
}

// Inequality operator
bool Pose2D::operator!=(const Pose2D& other) const {
    return !(*this == other);
}

// String representation
std::string Pose2D::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "Pose2D(x=" << x_ << ", y=" << y_ << ", theta=" << theta_ << ")";
    return oss.str();
}

// Copy method
Pose2D Pose2D::copy() const {
    return Pose2D(x_, y_, theta_);
}

// Convert to vector
std::vector<double> Pose2D::asList() const {
    return {x_, y_, theta_};
}

// Distance calculation
double Pose2D::distanceTo(const Pose2D& other) const {
    double dx = x_ - other.x_;
    double dy = y_ - other.y_;
    return std::sqrt(dx * dx + dy * dy);
}

// Angle calculation
double Pose2D::angleTo(const Pose2D& other) const {
    return std::atan2(other.y_ - y_, other.x_ - x_);
}

// Translation
void Pose2D::translate(double dx, double dy) {
    x_ += dx;
    y_ += dy;
}

// Rotation
void Pose2D::rotate(double dtheta) {
    theta_ += dtheta;
    theta_ = normalizeAngle(theta_);
}

// Combined transformation
void Pose2D::transform(double dx, double dy, double dtheta) {
    translate(dx, dy);
    rotate(dtheta);
}

// Convert to homogeneous matrix
std::vector<std::vector<double>> Pose2D::toHomogeneousMatrix() const {
    double cosTheta = std::cos(theta_);
    double sinTheta = std::sin(theta_);
    
    return {
        {cosTheta, -sinTheta, x_},
        {sinTheta,  cosTheta, y_},
        {0.0,       0.0,      1.0}
    };
}

// Create from homogeneous matrix
Pose2D Pose2D::fromHomogeneousMatrix(const std::vector<std::vector<double>>& matrix) {
    if (matrix.size() != 3 || matrix[0].size() != 3 || 
        matrix[1].size() != 3 || matrix[2].size() != 3) {
        throw std::invalid_argument("Matrix must be 3x3");
    }
    
    double x = matrix[0][2];
    double y = matrix[1][2];
    double theta = std::atan2(matrix[1][0], matrix[0][0]);
    
    return Pose2D(x, y, theta);
}

// Normalize angle to [-pi, pi]
double Pose2D::normalizeAngle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

// Get theta in degrees
double Pose2D::getThetaDegrees() const {
    return theta_ * 180.0 / PI;
}

// Set theta in degrees
void Pose2D::setThetaDegrees(double thetaDegrees) {
    theta_ = thetaDegrees * PI / 180.0;
}



// New method for Pose2D in pose.h: 
// Give it the base pose and a pose to fill in as result
void Pose2D::relative_pose_to(const Pose2D& base_pose, Pose2D& result){
	/* cout <<  this->x_ << " " <<this->y_ << " " <<this->theta_ << "\n" ; */
	/* cout << base_pose << endl; */
    result.x_ = (this->x_- base_pose.x_)*(cos(base_pose.theta_))+(this->y_- base_pose.y_)*(sin(base_pose.theta_));
    result.y_ = -(this->x_- base_pose.x_)*(sin(base_pose.theta_))+(this->y_- base_pose.y_)*(cos(base_pose.theta_));
    result.theta_ = this->theta_-base_pose.theta_;
}

// Stream output operator
std::ostream& operator<<(std::ostream& os, const Pose2D& pose) {
    os << pose.toString();
    return os;
}


// Helper functions that could go into class definition:
//
Pose2D invert_pose(const Pose2D& pose){
	return Pose2D {-pose.x_*cos(pose.theta_)-pose.y_*sin(pose.theta_),pose.x_*sin(pose.theta_)-pose.y_*cos(pose.theta_),-pose.theta_};
}


// Move by a transform defined locally to the original frame
void move_pose_local(Pose2D& pose, const Pose2D& motion){
	double cos_angle = cos(pose.theta_);
	double sin_angle = sin(pose.theta_);
	pose.x_ = pose.x_ + motion.x_* cos_angle  - motion.y_ * sin_angle;
	pose.y_ = pose.y_ + motion.x_* sin_angle  + motion.y_ * cos_angle;
	pose.theta_+=motion.theta_;
}

