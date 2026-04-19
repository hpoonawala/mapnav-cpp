#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <iomanip>
#include <tuple>
#include <Eigen/Dense>
#include "slam_posegraph.h"
#include "scan_match_11.h"
#include "pose.h"


/**
 * @brief A 2D occupancy grid for robotics mapping and path planning
 * 
 * This class maintains a probabilistic occupancy grid using log-odds representation.
 * It supports LiDAR scan integration, obstacle inflation, and A* path planning.
 * The grid uses a coordinate system where (0,0) is at the center.
 */
class OccupancyGrid {
private:
    double width, height, resolution;
    int grid_width, grid_height;
    std::vector<std::vector<double>> grid;           // Log-odds representation
    std::vector<std::vector<bool>> inflated_grid;    // Inflated obstacles for path planning
    
    // Hash function for grid coordinates in A* search
    struct PairHash {
        size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };
    
public:
    /**
     * @brief Constructor: Initialize an occupancy grid
     * @param w Width of the grid in meters
     * @param h Height of the grid in meters  
     * @param res Meters per grid cell (resolution)
     */
    OccupancyGrid(double w, double h, double res);
    
    /**
     * @brief Clear the grid (restart from blank map)
     */
    void clear();
    
    /**
     * @brief Convert world coordinates to grid indices
     * @param x World x coordinate in meters
     * @param y World y coordinate in meters
     * @return Grid indices (i, j)
     */
    std::pair<int, int> worldToGrid(double x, double y) const;
    
    /**
     * @brief Convert grid indices to world coordinates
     * @param i Grid column index
     * @param j Grid row index
     * @return World coordinates (x, y) in meters
     */
    std::pair<double, double> gridToWorld(int i, int j) const;
    
    /**
     * @brief Sigmoid function for probability conversion
     * @param x Log-odds value
     * @return Probability value between 0 and 1
     */
    double sigmoid(double x) const;
    
    /**
     * @brief Update occupancy grid with LiDAR scan data
     * @param lidar_x Vector of x coordinates in LiDAR frame
     * @param lidar_y Vector of y coordinates in LiDAR frame
     * @param robot_x Robot x position in world frame
     * @param robot_y Robot y position in world frame
     * @param robot_theta Robot orientation in radians
     */
    void updateWithScan(const std::vector<double>& lidar_x, 
                       const std::vector<double>& lidar_y, 
                       double robot_x, double robot_y, double robot_theta);
    
    void updateWithScan(const Scan& scan, const Pose& pose);
    /**
     * @brief Mark free space between two points using ray tracing
     * @param x1 Start point x coordinate
     * @param y1 Start point y coordinate
     * @param x2 End point x coordinate
     * @param y2 End point y coordinate
     */
    void markFreeSpaceVectorized(double x1, double y1, double x2, double y2);
    
    /**
     * @brief Convert log-odds grid to probability grid
     * @return 2D vector of probability values (0-1, probability of being free space)
     */
    std::vector<std::vector<double>> probabilityGrid() const;
    
    /**
     * @brief Check if a grid cell is occupied based on probability threshold
     * @param i Grid column index
     * @param j Grid row index
     * @param threshold Probability threshold (default 0.27)
     * @return True if cell is occupied
     */
    bool isOccupied(int i, int j, double threshold = 0.27) const;
    
    /**
     * @brief Inflate obstacles based on robot radius for safe path planning
     * @param robot_radius Robot radius in meters
     */
    void inflateObstacles(double robot_radius);
    
    /**
     * @brief Plan a path from start to goal using A* search algorithm
     * @param start Start position (x, y) in world coordinates
     * @param goal Goal position (x, y) in world coordinates
     * @return Vector of waypoints in world coordinates, empty if no path found
     */
    std::vector<std::pair<double, double>> planPath(const std::pair<double, double>& start, 
                                                   const std::pair<double, double>& goal);
    
    /**
     * @brief Write grid data to text file for analysis
     * @param filename Output filename
     * @param robot_x Current robot x position (for metadata)
     * @param robot_y Current robot y position (for metadata)
     */
    void writeGridToFile(const std::string& filename, double robot_x, double robot_y) const;
    
    /**
     * @brief Write probability grid as PGM image file (grayscale)
     * @param filename Output filename (.pgm extension recommended)
     */
    void writePGMFile(const std::string& filename) const;
    
    /**
     * @brief Print grid statistics to console
     */
    void printStats() const;
    
    /**
     * @brief Get grid dimensions in cells
     * @return Pair of (width, height) in grid cells
     */
    std::pair<int, int> getGridSize() const;
    
    /**
     * @brief Get grid resolution
     * @return Resolution in meters per cell
     */
    double getResolution() const;
    
    /**
     * @brief Get grid dimensions in meters
     * @return Pair of (width, height) in meters
     */
    std::pair<double, double> getWorldSize() const;
    
    /**
     * @brief Get direct access to the log-odds grid (read-only)
     * @return Const reference to the grid
     */
    const std::vector<std::vector<double>>& getGrid() const;
    
    /**
     * @brief Get direct access to the inflated grid (read-only)
     * @return Const reference to the inflated grid
     */
    const std::vector<std::vector<bool>>& getInflatedGrid() const;
    
    /**
     * @brief Check if grid coordinates are valid
     * @param i Grid column index
     * @param j Grid row index
     * @return True if coordinates are within grid bounds
     */
    bool isValidGridCoord(int i, int j) const;
    
    /**
     * @brief Check if world coordinates are within grid bounds
     * @param x World x coordinate
     * @param y World y coordinate
     * @return True if coordinates are within grid bounds
     */
    bool isValidWorldCoord(double x, double y) const;
    
    /**
     * @brief Get probability value at grid coordinates
     * @param i Grid column index
     * @param j Grid row index
     * @return Probability value (0-1), or -1 if coordinates invalid
     */
    double getProbability(int i, int j) const;
    
    /**
     * @brief Get log-odds value at grid coordinates
     * @param i Grid column index
     * @param j Grid row index
     * @return Log-odds value, or 0 if coordinates invalid
     */
    double getLogOdds(int i, int j) const;
    
    /**
     * @brief Set log-odds value at grid coordinates
     * @param i Grid column index
     * @param j Grid row index
     * @param value Log-odds value to set
     * @return True if successful, false if coordinates invalid
     */
    bool setLogOdds(int i, int j, double value);
    
    /**
     * @brief Update a single grid cell with log-odds increment
     * @param i Grid column index
     * @param j Grid row index
     * @param log_odds_update Log-odds increment to add
     * @return True if successful, false if coordinates invalid
     */
    bool updateCell(int i, int j, double log_odds_update);
};

#endif // OCCUPANCY_GRID_H
