// ============================================================================
// slam_posegraph.cpp - Implementation file
// ============================================================================

#include "slam_posegraph.h"
#include "scan_match_11.h"
#include <stdexcept>
#include <iostream>

// Pose implementation
Pose::Pose() : x(0.0), y(0.0), theta(0.0) {}

Pose::Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

double Pose::operator[](int idx) const {
    if (idx == 0) return x;
    if (idx == 1) return y;
    return theta;
}

// Helper functions implementation
Eigen::Vector3d invert_transform(const Eigen::Vector3d& pose) {
    double dx = pose[0];
    double dy = pose[1];
    double dt = pose[2];
    return Eigen::Vector3d(
        -dx * std::cos(dt) - dy * std::sin(dt),
        dx * std::sin(dt) - dy * std::cos(dt),
        -dt
    );
}

Eigen::Vector2d rotated_relative_position(double dx, double dy, double theta) {
    double dxR = dx * std::cos(theta) - dy * std::sin(theta);
    double dyR = dx * std::sin(theta) + dy * std::cos(theta);
    return Eigen::Vector2d(dxR, dyR);
}

Eigen::Vector3d relative_transform(const Pose& from_pose, const Pose& to_pose) {
    double dx = (to_pose.x - from_pose.x) * std::cos(from_pose.theta) + 
                (to_pose.y - from_pose.y) * std::sin(from_pose.theta);
    double dy = -(to_pose.x - from_pose.x) * std::sin(from_pose.theta) + 
                (to_pose.y - from_pose.y) * std::cos(from_pose.theta);
    double dt = to_pose.theta - from_pose.theta;
    return Eigen::Vector3d(dx, dy, dt);
}

std::string hash_scans(const Eigen::MatrixXd& scan1, const Eigen::MatrixXd& scan2) {
    std::ostringstream oss;
    int subset_size = std::min({50, static_cast<int>(scan1.rows()), static_cast<int>(scan2.rows())});
    
    for (int i = 0; i < subset_size; ++i) {
        for (int j = 0; j < scan1.cols(); ++j) {
            oss << std::fixed << std::setprecision(6) << scan1(i, j) << ",";
        }
    }
    oss << "|";
    for (int i = 0; i < subset_size; ++i) {
        for (int j = 0; j < scan2.cols(); ++j) {
            oss << std::fixed << std::setprecision(6) << scan2(i, j) << ",";
        }
    }
    return oss.str();
}

// ScanMatchCache::CacheKey implementation
bool ScanMatchCache::CacheKey::operator<(const CacheKey& other) const {
    if (scan_hash != other.scan_hash) return scan_hash < other.scan_hash;
    if (std::abs(grid_size - other.grid_size) > 1e-9) return grid_size < other.grid_size;
    return init_params.norm() < other.init_params.norm();
}

// ScanMatchCache implementation
ScanMatchCache::ScanMatchCache(size_t max_size) : max_size_(max_size) {}

bool ScanMatchCache::get(const Eigen::MatrixXd& scan1, const Eigen::MatrixXd& scan2,
                         double grid_size, const Eigen::Vector3d& init_params,
                         CacheValue& result) {
    CacheKey key{hash_scans(scan1, scan2), grid_size, init_params};
    
    auto it = cache_.find(key);
    if (it != cache_.end()) {
        access_count_[key]++;
        result = it->second;
        return true;
    }
    return false;
}

void ScanMatchCache::put(const Eigen::MatrixXd& scan1, const Eigen::MatrixXd& scan2,
                         double grid_size, const Eigen::Vector3d& init_params,
                         const CacheValue& result) {
    if (cache_.size() >= max_size_) {
        auto lru = std::min_element(access_count_.begin(), access_count_.end(),
            [](const std::pair<const CacheKey, int>& a, const std::pair<const CacheKey, int>& b) { 
                return a.second < b.second; 
            });
        cache_.erase(lru->first);
        access_count_.erase(lru->first);
    }
    
    CacheKey key{hash_scans(scan1, scan2), grid_size, init_params};
    cache_[key] = result;
    access_count_[key] = 1;
}

// PoseGraph implementation
PoseGraph::PoseGraph(NDTScanMatcher& matcher, const Pose& initial_pose, double grid_size)
    : matcher (matcher), initial_pose_(initial_pose), grid_size_(grid_size) {}

std::vector<std::vector<int>> PoseGraph::build_graph_edges(int n, int first_ind, int ind_interval) {
    std::vector<std::vector<int>> edges;
    int final_ind = n;
    
    for (int i = first_ind; i < final_ind - 2 * ind_interval; i += ind_interval) {
        edges.push_back({i, i + ind_interval});
        edges.push_back({i, i + 2 * ind_interval});
    }
    
    int last_index = ((final_ind - 2 * ind_interval) / ind_interval) * ind_interval;
    
    if (last_index != n - 1 && last_index >= ind_interval) {
        edges.push_back({last_index - ind_interval, n - 1});
    } else if (last_index != n && last_index == 0) {
        edges.push_back({last_index, n - 1});
    }
    
    return edges;
}

std::vector<int> PoseGraph::get_nodes_from_edges(const std::vector<std::vector<int>>& edges,
                                                  int n, int ind_interval) {
    std::set<int> nodes_set;
    
    for (const auto& edge : edges) {
        nodes_set.insert(edge[0]);
        nodes_set.insert(edge[1]);
    }
    
    for (int i = 0; i < n; i += ind_interval) {
        nodes_set.insert(i);
    }
    
    if (nodes_set.find(n - 1) == nodes_set.end()) {
        nodes_set.insert(n - 1);
    }
    
    return std::vector<int>(nodes_set.begin(), nodes_set.end());
}

ScanMatchCache::CacheValue PoseGraph::cached_scan_match(
    const Eigen::MatrixXd& scan1,
    const Eigen::MatrixXd& scan2,
    double grid_size,
    const Eigen::Vector3d& init_params,
    int max_iters) {
    
    ScanMatchCache::CacheValue cached_result;
    if (cache_.get(scan1, scan2, grid_size, init_params, cached_result)) {
        return cached_result;
    }
    
	Pose2D result_p2d;
		Matrix3d hessian;
    this->matcher.ndtScanMatchHP(
        scan2, scan1, grid_size, result_p2d,hessian, max_iters, 1e-6,
        init_params[0], init_params[1], init_params[2],
         new bool(false)
    );
    
    ScanMatchCache::CacheValue result;
    result.match_result = result_p2d;
    result.hessian = hessian;
    
    cache_.put(scan1, scan2, grid_size, init_params, result);
    return result;
}

void PoseGraph::build_sparse_system(
    const std::vector<std::vector<int>>& edges,
    const std::vector<int>& nodes,
    const std::vector<Eigen::Vector3d>& relative_poses,
    Eigen::SparseMatrix<double>& A_mat,
    Eigen::VectorXd& b_vec,
    std::map<int, int>& vertex_dict) {
    
    int n_edges = edges.size();
    int n_nodes = nodes.size();
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        vertex_dict[nodes[i]] = i;
    }
    
    int n_constraints = 3 * n_edges + 3;
    int n_variables = 3 * n_nodes;
    
    A_mat.resize(n_constraints, n_variables);
    b_vec.resize(n_constraints);
    b_vec.setZero();
    
    std::vector<Eigen::Triplet<double>> triplets;
    
    for (size_t i = 0; i < edges.size(); ++i) {
        int node1 = edges[i][0];
        int node2 = edges[i][1];
        int idx1 = vertex_dict[node1];
        int idx2 = vertex_dict[node2];
        
        for (int j = 0; j < 3; ++j) {
            int row_idx = 3 * i + j;
            int col_idx1 = 3 * idx1 + j;
            int col_idx2 = 3 * idx2 + j;
            
            triplets.push_back(Eigen::Triplet<double>(row_idx, col_idx2, 1.0));
            triplets.push_back(Eigen::Triplet<double>(row_idx, col_idx1, -1.0));
            b_vec[row_idx] = relative_poses[i][j];
        }
    }
    
    int anchor_start = 3 * n_edges;
    triplets.push_back(Eigen::Triplet<double>(anchor_start, 0, 1.0));
    triplets.push_back(Eigen::Triplet<double>(anchor_start + 1, 1, 1.0));
    triplets.push_back(Eigen::Triplet<double>(anchor_start + 2, 2, 1.0));
    
    b_vec[anchor_start] = initial_pose_.x;
    b_vec[anchor_start + 1] = initial_pose_.y;
    b_vec[anchor_start + 2] = initial_pose_.theta;
    
    A_mat.setFromTriplets(triplets.begin(), triplets.end());
}

Eigen::VectorXd PoseGraph::solve_pose_graph(const Eigen::SparseMatrix<double>& A_mat,
                                            const Eigen::VectorXd& b_vec) {
    Eigen::SparseMatrix<double> AtA = A_mat.transpose() * A_mat;
    Eigen::VectorXd Atb = A_mat.transpose() * b_vec;
    
    double regularization = 1e-6;
    for (int i = 0; i < AtA.rows(); ++i) {
        AtA.coeffRef(i, i) += regularization;
    }
    
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(AtA);
    
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Sparse solver decomposition failed");
    }
    
    return solver.solve(Atb);
}

std::vector<Pose> PoseGraph::update_poses_efficiently(
    const std::vector<Pose>& odom_poses,
    const Eigen::VectorXd& solution,
    const std::vector<int>& nodes,
    const std::map<int, int>& vertex_dict) {
    
    std::vector<Pose> updated_poses = odom_poses;
    
    for (int node : nodes) {
        int idx = vertex_dict.at(node);
        updated_poses[node].x = solution[3 * idx];
        updated_poses[node].y = solution[3 * idx + 1];
        updated_poses[node].theta = solution[3 * idx + 2];
    }
    
    return updated_poses;
}

std::vector<std::vector<int>>& PoseGraph::get_previous_graph() {
    return previous_graph_;
}

std::map<std::pair<int, int>, Eigen::Vector3d>& PoseGraph::get_previous_results() {
    return previous_results_;
}

double PoseGraph::get_grid_size() const {
    return grid_size_;
}

// Main mapping function implementation
std::pair<std::vector<Pose>, std::vector<int>> mapping_optimized(
    const std::vector<Eigen::MatrixXd>& scanlist,
    const std::vector<Pose>& odom_poses,
    PoseGraph& posegraph,
    int ind_interval) {
    
    int n = scanlist.size();
    
    auto current_edges = posegraph.build_graph_edges(n, 0, ind_interval);
    auto current_nodes = posegraph.get_nodes_from_edges(current_edges, n, ind_interval);
    
    std::set<std::pair<int, int>> current_edge_set;
    for (const auto& edge : current_edges) {
        int a = std::min(edge[0], edge[1]);
        int b = std::max(edge[0], edge[1]);
        current_edge_set.insert({a, b});
    }
    
    std::vector<std::vector<int>> new_edges;
    auto& prev_graph = posegraph.get_previous_graph();
    
    if (prev_graph.empty()) {
        new_edges = current_edges;
        std::cout << "First run: computing " << new_edges.size() << " edges" << std::endl;
    } else {
        std::set<std::pair<int, int>> previous_edge_set;
        for (const auto& edge : prev_graph) {
            int a = std::min(edge[0], edge[1]);
            int b = std::max(edge[0], edge[1]);
            previous_edge_set.insert({a, b});
        }
        
        for (const auto& edge_pair : current_edge_set) {
            if (previous_edge_set.find(edge_pair) == previous_edge_set.end()) {
                new_edges.push_back({edge_pair.first, edge_pair.second});
            }
        }
        std::cout << "Incremental: computing " << new_edges.size() 
                  << " new edges out of " << current_edges.size() << " total" << std::endl;
    }
    
    auto& previous_results = posegraph.get_previous_results();
    
    for (const auto& edge : new_edges) {
        int ind_one = edge[0];
        int ind_two = edge[1];
        
        Eigen::Vector3d rel_trans = relative_transform(odom_poses[ind_one], odom_poses[ind_two]);
        Eigen::Vector3d inv_trans = invert_transform(rel_trans);
        
        auto cached_result = posegraph.cached_scan_match(
            scanlist[ind_one], scanlist[ind_two],
            posegraph.get_grid_size(), inv_trans, 500
        );
        
        Eigen::Vector3d manual_pose(
            cached_result.match_result[0],
            cached_result.match_result[1],
            cached_result.match_result[2]
        );
        
        Eigen::Vector3d inv_pose = invert_transform(manual_pose);
        Eigen::Vector2d rotated = rotated_relative_position(
            inv_pose[0], inv_pose[1], odom_poses[ind_one].theta
        );
        
        int a = std::min(ind_one, ind_two);
        int b = std::max(ind_one, ind_two);
        previous_results[{a, b}] = Eigen::Vector3d(rotated[0], rotated[1], -manual_pose[2]);
    }
    
    std::vector<Eigen::Vector3d> relative_poses;
    for (const auto& edge : current_edges) {
        int a = std::min(edge[0], edge[1]);
        int b = std::max(edge[0], edge[1]);
        
        auto it = previous_results.find({a, b});
        if (it != previous_results.end()) {
            Eigen::Vector3d stored_pose = it->second;
            if (edge[0] > edge[1]) {
                stored_pose = -stored_pose;
            }
            relative_poses.push_back(stored_pose);
        } else {
            std::cout << "Warning: Missing result for edge [" << edge[0] << ", " << edge[1] << "]" << std::endl;
            relative_poses.push_back(Eigen::Vector3d::Zero());
        }
    }
    
    prev_graph = current_edges;
    
    Eigen::SparseMatrix<double> A_mat;
    Eigen::VectorXd b_vec;
    std::map<int, int> vertex_dict;
    
    posegraph.build_sparse_system(current_edges, current_nodes, relative_poses,
                                   A_mat, b_vec, vertex_dict);
    
    Eigen::VectorXd solution = posegraph.solve_pose_graph(A_mat, b_vec);
    
    auto updated_poses = posegraph.update_poses_efficiently(
        odom_poses, solution, current_nodes, vertex_dict
    );
    
    return {updated_poses, current_nodes};
}
