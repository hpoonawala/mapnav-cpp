#include "OccupancyGrid.h"

// Constructor: Initialize occupancy grid
OccupancyGrid::OccupancyGrid(double w, double h, double res) 
    : width(w), height(h), resolution(res) {
    grid_width = static_cast<int>(width / resolution);
    grid_height = static_cast<int>(height / resolution);
    
    // Initialize grid with zeros (unknown state)
    grid.resize(grid_width, std::vector<double>(grid_height, 0.0));
    inflated_grid.resize(grid_width, std::vector<bool>(grid_height, false));
}

// Clear the grid (restart from blank map)
void OccupancyGrid::clear() {
    for (auto& row : grid) {
        std::fill(row.begin(), row.end(), 0.0);
    }
    for (auto& row : inflated_grid) {
        std::fill(row.begin(), row.end(), false);
    }
}

// Convert world coordinates to grid indices
std::pair<int, int> OccupancyGrid::worldToGrid(double x, double y) const {
    int j = static_cast<int>((height / 2 + y) / resolution);
    int i = static_cast<int>((x + width / 2) / resolution);
    return std::make_pair(i, j);
}

// Convert grid indices to world coordinates
std::pair<double, double> OccupancyGrid::gridToWorld(int i, int j) const {
    double y = -height / 2 + j * resolution;
    double x = i * resolution - width / 2;
    return std::make_pair(x, y);
}

// Sigmoid function for probability conversion
double OccupancyGrid::sigmoid(double x) const {
    return 1.0 / (1.0 + std::exp(-x));
}

// Update occupancy grid with LiDAR scan data
void OccupancyGrid::updateWithScan(const std::vector<double>& lidar_x, 
                   const std::vector<double>& lidar_y, 
                   double robot_x, double robot_y, double robot_theta) {
    
    // Placeholder: Insert scan matching function call here to refine robot pose if needed
    // auto refined_pose = scanMatchingFunction(lidar_x, lidar_y, robot_x, robot_y, robot_theta);
    // robot_x = refined_pose.x; robot_y = refined_pose.y; robot_theta = refined_pose.theta;
    
    // Process each LiDAR point
    size_t min_size = std::min(lidar_x.size(), lidar_y.size());
    for (size_t i = 0; i < min_size; ++i) {
        double world_x = lidar_x[i];
        double world_y = lidar_y[i];
        
        std::pair<int, int> grid_coords = worldToGrid(world_x, world_y);
        int grid_x = grid_coords.first;
        int grid_y = grid_coords.second;
        
        // Mark free space between robot and obstacle
        markFreeSpaceVectorized(robot_x, robot_y, world_x, world_y);
        
        // Mark obstacle and surrounding cells as occupied
        if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
            for (int di = -1; di <= 1; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    int ni = grid_x + di;
                    int nj = grid_y + dj;
                    if (ni >= 0 && ni < grid_width && nj >= 0 && nj < grid_height) {
                        grid[ni][nj] = -1.0; // Mark as occupied
                        if (grid[ni][nj] < -1.0) {
                            grid[ni][nj] = -1.0;
                        }
                    }
                }
            }
        }
    }
    
    // Inflate obstacles based on robot radius
    //inflateObstacles(0.5);
}

// Mark free space between two points using vectorized approach
void OccupancyGrid::markFreeSpaceVectorized(double x1, double y1, double x2, double y2) {
    std::pair<int, int> coords1 = worldToGrid(x1, y1);
    std::pair<int, int> coords2 = worldToGrid(x2, y2);
    int grid_x1 = coords1.first;
    int grid_y1 = coords1.second;
    int grid_x2 = coords2.first;
    int grid_y2 = coords2.second;
    
    int dx = std::abs(grid_x2 - grid_x1);
    int dy = std::abs(grid_y2 - grid_y1);
    int num_points = std::max(dx, dy) + 1;
    
    if (num_points <= 1) {
        if (grid_x1 >= 0 && grid_x1 < grid_width && grid_y1 >= 0 && grid_y1 < grid_height) {
            grid[grid_x1][grid_y1] += 0.176 * 2;
        }
        return;
    }
    
    // Generate line points
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        int x_coord = static_cast<int>(grid_x1 + t * (grid_x2 - grid_x1));
        int y_coord = static_cast<int>(grid_y1 + t * (grid_y2 - grid_y1));
        
        // Check bounds and update
        if (x_coord >= 0 && x_coord < grid_width && y_coord >= 0 && y_coord < grid_height) {
            grid[x_coord][y_coord] += 0.176 * 2;
        }
    }
}


// Update occupancy grid with LiDAR scan data
void OccupancyGrid::updateWithScan(const Scan& scan, const Pose& pose) {
    
    // Placeholder: Insert scan matching function call here to refine robot pose if needed
    // auto refined_pose = scanMatchingFunction(lidar_x, lidar_y, robot_x, robot_y, robot_theta);
    // robot_x = refined_pose.x; robot_y = refined_pose.y; robot_theta = refined_pose.theta;
    
    // Process each LiDAR point
    size_t min_size = scan.rows();
    for (size_t i = 0; i < min_size; ++i) {
        double world_x = scan(i,0);
        double world_y = scan(i,1);
        
        std::pair<int, int> grid_coords = worldToGrid(world_x, world_y);
        int grid_x = grid_coords.first;
        int grid_y = grid_coords.second;
        
        // Mark free space between robot and obstacle
        markFreeSpaceVectorized(pose[0], pose[1], world_x, world_y);
        
        // Mark obstacle and surrounding cells as occupied
        if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
            for (int di = -1; di <= 1; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    int ni = grid_x + di;
                    int nj = grid_y + dj;
                    if (ni >= 0 && ni < grid_width && nj >= 0 && nj < grid_height) {
                        grid[ni][nj] = -1.0; // Mark as occupied
                        if (grid[ni][nj] < -1.0) {
                            grid[ni][nj] = -1.0;
                        }
                    }
                }
            }
        }
    }
    
    // Inflate obstacles based on robot radius
    //inflateObstacles(0.1);
}

// Convert log-odds grid to probability grid
std::vector<std::vector<double>> OccupancyGrid::probabilityGrid() const {
    std::vector<std::vector<double>> prob_grid(grid_width, std::vector<double>(grid_height));
    for (int i = 0; i < grid_width; ++i) {
        for (int j = 0; j < grid_height; ++j) {
            prob_grid[i][j] = sigmoid(grid[i][j]);
        }
    }
    return prob_grid;
}

// Check if a grid cell is occupied based on probability threshold
bool OccupancyGrid::isOccupied(int i, int j, double threshold) const {
    if (i < 0 || i >= grid_width || j < 0 || j >= grid_height) return true;
    double prob = sigmoid(grid[i][j]);
    return prob < threshold;
}

// Inflate obstacles based on robot radius
void OccupancyGrid::inflateObstacles(double robot_radius) {
    int cell_radius = static_cast<int>(robot_radius / resolution);
    //std::vector<std::vector<double>> prob_grid = probabilityGrid();
    
    // Reset inflated grid
    for (size_t i = 0; i < inflated_grid.size(); ++i) {
        std::fill(inflated_grid[i].begin(), inflated_grid[i].end(), false);
    }
    
    // Apply maximum filter (morphological dilation)
    for (int i = 0; i < grid_width; ++i) {
        for (int j = 0; j < grid_height; ++j) {
            if (grid[i][j] < -0.2) { // Cell is occupied
                // Inflate around this cell
                for (int di = -cell_radius; di <= cell_radius; ++di) {
                    for (int dj = -cell_radius; dj <= cell_radius; ++dj) {
                        int ni = i + di;
                        int nj = j + dj;
                        if (ni >= 0 && ni < grid_width && nj >= 0 && nj < grid_height) {
                            inflated_grid[ni][nj] = true;
                        }
                    }
                }
            }
        }
    }
}

// A* path planning from start to goal
std::vector<std::pair<double, double>> OccupancyGrid::planPath(const std::pair<double, double>& start, 
                                               const std::pair<double, double>& goal) {
    std::pair<int, int> start_coords = worldToGrid(start.first, start.second);
    std::pair<int, int> goal_coords = worldToGrid(goal.first, goal.second);
    int start_i = start_coords.first;
    int start_j = start_coords.second;
    int goal_i = goal_coords.first;
    int goal_j = goal_coords.second;
    
    // Priority queue for A* (f_score, i, j)
    std::priority_queue<std::tuple<double, int, int>, 
                      std::vector<std::tuple<double, int, int>>,
                      std::greater<std::tuple<double, int, int>>> open_set;
    
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> came_from;
    std::unordered_map<std::pair<int, int>, double, PairHash> g_score;
    
    open_set.push(std::make_tuple(0.0, start_i, start_j));
    g_score[std::make_pair(start_i, start_j)] = 0.0;
    
    // Heuristic function (Euclidean distance)
    auto heuristic = [](int i1, int j1, int i2, int j2) -> double {
        return std::sqrt((i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2));
    };
    
    while (!open_set.empty()) {
        std::tuple<double, int, int> current_tuple = open_set.top();
        open_set.pop();
        double f_score = std::get<0>(current_tuple);
        int current_i = std::get<1>(current_tuple);
        int current_j = std::get<2>(current_tuple);
        
        if (current_i == goal_i && current_j == goal_j) {
            // Reconstruct path
            std::vector<std::pair<int, int>> grid_path;
            std::pair<int, int> current = std::make_pair(current_i, current_j);
            
            while (came_from.find(current) != came_from.end()) {
                grid_path.push_back(current);
                current = came_from[current];
            }
            grid_path.push_back(std::make_pair(start_i, start_j));
            std::reverse(grid_path.begin(), grid_path.end());
            
            // Convert to world coordinates and stop at unvisited cells
            std::vector<std::pair<double, double>> result;
            for (size_t k = 0; k < grid_path.size(); ++k) {
                int i = grid_path[k].first;
                int j = grid_path[k].second;
                result.push_back(gridToWorld(i, j));
                if (grid[i][j] == 0.0) { // Unvisited cell (log-odds of 0.5)
                    break;
                }
            }
            return result;
        }
        
        // Explore neighbors
        for (int di = -1; di <= 1; ++di) {
            for (int dj = -1; dj <= 1; ++dj) {
                if (di == 0 && dj == 0) continue;
                
                int ni = current_i + di;
                int nj = current_j + dj;
                
                if (ni >= 0 && ni < grid_width && nj >= 0 && nj < grid_height) {
                    // Skip inflated obstacles
                    if (inflated_grid[ni][nj]) continue;
                    
                    double tentative_g = g_score[std::make_pair(current_i, current_j)] + 
                                       std::sqrt(di * di + dj * dj);
                    
                    std::pair<int, int> neighbor = std::make_pair(ni, nj);
                    if (g_score.find(neighbor) == g_score.end() || 
                        tentative_g < g_score[neighbor]) {
                        
                        g_score[neighbor] = tentative_g;
                        double f = tentative_g + heuristic(ni, nj, goal_i, goal_j);
                        open_set.push(std::make_tuple(f, ni, nj));
                        came_from[neighbor] = std::make_pair(current_i, current_j);
                    }
                }
            }
        }
    }
    
    return std::vector<std::pair<double, double>>(); // No path found
}

// Write grid data to file for visualization
void OccupancyGrid::writeGridToFile(const std::string& filename, double robot_x, double robot_y) const {
    std::ofstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    
    // Write header with grid information
    file << "# Occupancy Grid Data\n";
    file << "# Width: " << width << " Height: " << height << " Resolution: " << resolution << "\n";
    file << "# Grid Size: " << grid_width << "x" << grid_height << "\n";
    file << "# Robot Position: (" << robot_x << ", " << robot_y << ")\n";
    file << "# Format: i j world_x world_y log_odds probability\n";
    
    for (int i = 0; i < grid_width; ++i) {
        for (int j = 0; j < grid_height; ++j) {
            std::pair<double, double> world_coords = gridToWorld(i, j);
            double world_x = world_coords.first;
            double world_y = world_coords.second;
            double prob = sigmoid(grid[i][j]);
            file << i << " " << j << " " 
                 << std::fixed << std::setprecision(6) 
                 << world_x << " " << world_y << " " 
                 << grid[i][j] << " " << prob << "\n";
        }
    }
    file.close();
    std::cout << "Grid data written to " << filename << std::endl;
}

// Write probability grid as PGM image file (grayscale)
void OccupancyGrid::writePGMFile(const std::string& filename) const {
    std::ofstream file(filename.c_str(), std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    
    // PGM header
    file << "P5\n";
    file << grid_width << " " << grid_height << "\n";
    file << "255\n";
    
    // Write pixel data (probability converted to grayscale)
    for (int j = grid_height - 1; j >= 0; --j) { // Flip vertically for correct orientation
        for (int i = 0; i < grid_width; ++i) {
            double prob = sigmoid(grid[i][j]);
            unsigned char pixel = static_cast<unsigned char>(prob * 255);
            file.write(reinterpret_cast<const char*>(&pixel), 1);
        }
    }
    file.close();
    std::cout << "PGM image written to " << filename << std::endl;
}

// Print grid statistics
void OccupancyGrid::printStats() const {
    if (grid.empty() || grid[0].empty()) {
        std::cout << "Grid is empty\n";
        return;
    }
    
    double min_val = grid[0][0], max_val = grid[0][0];
    int occupied_cells = 0, free_cells = 0, unknown_cells = 0;
    
    for (int i = 0; i < grid_width; ++i) {
        for (int j = 0; j < grid_height; ++j) {
            min_val = std::min(min_val, grid[i][j]);
            max_val = std::max(max_val, grid[i][j]);
            
            double prob = sigmoid(grid[i][j]);
            if (prob < 0.27) occupied_cells++;
            else if (prob > 0.73) free_cells++;
            else unknown_cells++;
        }
    }
    
    std::cout << "Grid Statistics:\n";
    std::cout << "  Size: " << grid_width << "x" << grid_height << " cells\n";
    std::cout << "  World size: " << width << "x" << height << " meters\n";
    std::cout << "  Resolution: " << resolution << " m/cell\n";
    std::cout << "  Log-odds range: [" << min_val << ", " << max_val << "]\n";
    std::cout << "  Occupied cells: " << occupied_cells << "\n";
    std::cout << "  Free cells: " << free_cells << "\n";
    std::cout << "  Unknown cells: " << unknown_cells << "\n";
}

// Get grid dimensions in cells
std::pair<int, int> OccupancyGrid::getGridSize() const {
    return std::make_pair(grid_width, grid_height);
}

// Get grid resolution
double OccupancyGrid::getResolution() const {
    return resolution;
}

// Get grid dimensions in meters
std::pair<double, double> OccupancyGrid::getWorldSize() const {
    return std::make_pair(width, height);
}

// Get direct access to the log-odds grid (read-only)
const std::vector<std::vector<double>>& OccupancyGrid::getGrid() const {
    return grid;
}

// Get direct access to the inflated grid (read-only)
const std::vector<std::vector<bool>>& OccupancyGrid::getInflatedGrid() const {
    return inflated_grid;
}

// Check if grid coordinates are valid
bool OccupancyGrid::isValidGridCoord(int i, int j) const {
    return i >= 0 && i < grid_width && j >= 0 && j < grid_height;
}

// Check if world coordinates are within grid bounds
bool OccupancyGrid::isValidWorldCoord(double x, double y) const {
    return x >= -width/2 && x < width/2 && y >= -height/2 && y < height/2;
}

// Get probability value at grid coordinates
double OccupancyGrid::getProbability(int i, int j) const {
    if (!isValidGridCoord(i, j)) return -1.0;
    return sigmoid(grid[i][j]);
}

// Get log-odds value at grid coordinates
double OccupancyGrid::getLogOdds(int i, int j) const {
    if (!isValidGridCoord(i, j)) return 0.0;
    return grid[i][j];
}

// Set log-odds value at grid coordinates
bool OccupancyGrid::setLogOdds(int i, int j, double value) {
    if (!isValidGridCoord(i, j)) return false;
    grid[i][j] = value;
    return true;
}

// Update a single grid cell with log-odds increment
bool OccupancyGrid::updateCell(int i, int j, double log_odds_update) {
    if (!isValidGridCoord(i, j)) return false;
    grid[i][j] += log_odds_update;
    return true;
}
