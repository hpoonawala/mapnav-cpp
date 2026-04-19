// ============================================================================
// slam_posegraph.h - Header file
// ============================================================================
#ifndef SLAM_POSEGRAPH_H
#define SLAM_POSEGRAPH_H

#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "pose.h"
#include "scan_match_11.h"

// Forward declaration for scan matching
namespace scan_match {
    
    struct ScanMatchOutput {
        Pose2D match_result;
        Eigen::Matrix3d hessian;
    };
    
    // Implement this function in your scan matching module
    ScanMatchOutput ndt_scan_match_hp(
        const Eigen::MatrixXd& scan2,
        const Eigen::MatrixXd& scan1,
        double grid_size,
        double tx_init,
        double ty_init,
        double phi_init,
        int max_iters
    );
}

// Pose structure
struct Pose {
    double x;
    double y;
    double theta;
    
    Pose();
    Pose(double x_, double y_, double theta_);
    double operator[](int idx) const;
};

// Helper function declarations
Eigen::Vector3d invert_transform(const Eigen::Vector3d& pose);
Eigen::Vector2d rotated_relative_position(double dx, double dy, double theta);
Eigen::Vector3d relative_transform(const Pose& from_pose, const Pose& to_pose);
std::string hash_scans(const Eigen::MatrixXd& scan1, const Eigen::MatrixXd& scan2);

// Scan match cache
class ScanMatchCache {
public:
    struct CacheKey {
        std::string scan_hash;
        double grid_size;
        Eigen::Vector3d init_params;
        
        bool operator<(const CacheKey& other) const;
    };
    
    struct CacheValue {
        Pose2D match_result;
        Eigen::Matrix3d hessian;
    };
    
    ScanMatchCache(size_t max_size = 1000);
    
    bool get(const Eigen::MatrixXd& scan1, const Eigen::MatrixXd& scan2,
             double grid_size, const Eigen::Vector3d& init_params,
             CacheValue& result);
    
    void put(const Eigen::MatrixXd& scan1, const Eigen::MatrixXd& scan2,
             double grid_size, const Eigen::Vector3d& init_params,
             const CacheValue& result);
    
private:
    size_t max_size_;
    std::map<CacheKey, CacheValue> cache_;
    std::map<CacheKey, int> access_count_;
};

// Pose graph class
class PoseGraph {
public:
    PoseGraph(NDTScanMatcher& matcher, const Pose& initial_pose = Pose(), double grid_size = 2.0);
    
    std::vector<std::vector<int>> build_graph_edges(int n, int first_ind = 0, int ind_interval = 10);
    
    std::vector<int> get_nodes_from_edges(const std::vector<std::vector<int>>& edges,
                                          int n, int ind_interval = 10);
    
    ScanMatchCache::CacheValue cached_scan_match(
        const Eigen::MatrixXd& scan1,
        const Eigen::MatrixXd& scan2,
        double grid_size,
        const Eigen::Vector3d& init_params,
        int max_iters = 500);
    
    void build_sparse_system(
        const std::vector<std::vector<int>>& edges,
        const std::vector<int>& nodes,
        const std::vector<Eigen::Vector3d>& relative_poses,
        Eigen::SparseMatrix<double>& A_mat,
        Eigen::VectorXd& b_vec,
        std::map<int, int>& vertex_dict);
    
    Eigen::VectorXd solve_pose_graph(const Eigen::SparseMatrix<double>& A_mat,
                                     const Eigen::VectorXd& b_vec);
    
    std::vector<Pose> update_poses_efficiently(
        const std::vector<Pose>& odom_poses,
        const Eigen::VectorXd& solution,
        const std::vector<int>& nodes,
        const std::map<int, int>& vertex_dict);
    
    std::vector<std::vector<int>>& get_previous_graph();
    std::map<std::pair<int, int>, Eigen::Vector3d>& get_previous_results();
    double get_grid_size() const;
    
private:
	NDTScanMatcher matcher;
    ScanMatchCache cache_;
    std::vector<std::vector<int>> previous_graph_;
    std::map<std::pair<int, int>, Eigen::Vector3d> previous_results_;
    Pose initial_pose_;
    double grid_size_;
};

// Main mapping function
std::pair<std::vector<Pose>, std::vector<int>> mapping_optimized(
    const std::vector<Eigen::MatrixXd>& scanlist,
    const std::vector<Pose>& odom_poses,
    PoseGraph& posegraph,
    int ind_interval = 10);

#endif // SLAM_POSEGRAPH_H
