#ifndef POSE2D_H
#define POSE2D_H

#include <vector>
#include <string>

/**
 * A class representing a 2D pose with x, y coordinates and orientation angle theta.
 */
class Pose2D {

public:
    double x_;      ///< X coordinate
    double y_;      ///< Y coordinate  
    double theta_;  ///< Orientation angle in radians
    /**
     * Default constructor - initializes pose to origin with zero rotation.
     */
    Pose2D();

    /**
     * Constructor with parameters.
     * 
     * @param x X coordinate (default: 0.0)
     * @param y Y coordinate (default: 0.0)
     * @param theta Orientation angle in radians (default: 0.0)
     */
    Pose2D(double x, double y, double theta = 0.0);

    /**
     * Copy constructor.
     */
    Pose2D(const Pose2D& other);

    /**
     * Assignment operator.
     */
    Pose2D& operator=(const Pose2D& other);

    /**
     * Destructor.
     */
    ~Pose2D() = default;

    // Getters
    double getX() const { return x_; }
    double getY() const { return y_; }
    double getTheta() const { return theta_; }

    // Setters
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void setTheta(double theta) { theta_ = theta; }

    /**
     * Array-style access operator.
     * @param index 0 for x, 1 for y, 2 for theta
     * @return Reference to the coordinate
     */
    double operator[](int index) const;

    /**
     * Equality operator with small tolerance.
     */
    bool operator==(const Pose2D& other) const;

    /**
     * Inequality operator.
     */
    bool operator!=(const Pose2D& other) const;

    /**
     * String representation of the pose.
     */
    std::string toString() const;

    /**
     * Create a copy of the pose.
     * @return New Pose2D object with same values
     */
    Pose2D copy() const;

    /**
     * Convert pose to vector.
     * @return Vector containing [x, y, theta]
     */
    std::vector<double> asList() const;

    /**
     * Calculate Euclidean distance to another pose.
     * 
     * @param other Another pose
     * @return Euclidean distance
     */
    double distanceTo(const Pose2D& other) const;

    /**
     * Calculate angle from this pose to another pose.
     * 
     * @param other Another pose
     * @return Angle in radians
     */
    double angleTo(const Pose2D& other) const;

    /**
     * Translate the pose by given offsets.
     * 
     * @param dx X offset
     * @param dy Y offset
     */
    void translate(double dx, double dy);

    /**
     * Rotate the pose by given angle.
     * 
     * @param dtheta Rotation angle in radians
     */
    void rotate(double dtheta);

    /**
     * Apply translation and rotation to the pose.
     * 
     * @param dx X offset
     * @param dy Y offset
     * @param dtheta Rotation angle in radians
     */
    void transform(double dx, double dy, double dtheta);

    /**
     * Convert pose to 3x3 homogeneous transformation matrix.
     * 
     * @return 3x3 transformation matrix as vector of vectors
     */
    std::vector<std::vector<double>> toHomogeneousMatrix() const;

    /**
     * Create pose from 3x3 homogeneous transformation matrix.
     * 
     * @param matrix 3x3 transformation matrix as vector of vectors
     * @return New pose object
     */
    static Pose2D fromHomogeneousMatrix(const std::vector<std::vector<double>>& matrix);

    /**
     * Normalize angle to [-pi, pi] range.
     * 
     * @param angle Angle in radians
     * @return Normalized angle in [-pi, pi]
     */
    static double normalizeAngle(double angle);

    /**
     * Get orientation angle in degrees.
     * 
     * @return Angle in degrees
     */
    double getThetaDegrees() const;

    /**
     * Set orientation angle in degrees.
     * 
     * @param thetaDegrees Angle in degrees
     */
    void setThetaDegrees(double thetaDegrees);

	void relative_pose_to(const Pose2D&, Pose2D&);
};
Pose2D invert_pose(const Pose2D&);
void move_pose_local(Pose2D&, const Pose2D&);
// Stream output operator
std::ostream& operator<<(std::ostream& os, const Pose2D& pose);

#endif // POSE2D_H
