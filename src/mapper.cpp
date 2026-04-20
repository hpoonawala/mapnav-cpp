#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <chrono>
#include <stdio.h>
#include "../include/pose.h"
#include "../include/scan_match_11.h"
#include "../include/OccupancyGrid.h"
#include "../include/slam_posegraph.h"
#include "../include/mapper.h"
#include "../include/timer.h"

Mapper::Mapper() :	matcher {},
					posegraph {matcher, Pose(0.0,0.0,0.0), 0.85},
					grid {OccupancyGrid(10.0,10.0,0.02)},
					gridsize{0.85},
					curr_pose{Pose2D(0.0,0.0,0.0)}
					{};

Scan polarToCartesian(Scan& scan, int n_samples) {
	Scan cart_scan = Scan(n_samples, 2);
	for (int i = 0; i < n_samples; i++) {
		cart_scan(i,0) = scan(i,1) * cos(scan(i,0));
		cart_scan(i,1) = -scan(i,1) * sin(scan(i,0));
	}
	return cart_scan;
}

void Mapper::update_scans(Scan& scan_polar) {
	Scan scan = polarToCartesian(scan_polar, scan_polar.rows());
	int n = frame_history.size();
	if (n == 0) {
		frame_history.append({scan, Pose(curr_pose.x_, curr_pose.y_, curr_pose.theta_)});
	} else {
		Scan prev_scan = frame_history.last_scan();
		Pose2D result;
		Matrix3d hessian;
		Timer timer;
		matcher.ndtScanMatchHP(prev_scan, scan, gridsize, result, hessian, 60, 1e-6, 0.0, 0.0, 0.0, false);
		timer.mark("scan match"); timer.reset();
		move_pose_local(curr_pose, result);
		frame_history.append({scan, Pose(curr_pose.x_, curr_pose.y_, curr_pose.theta_)});
	}
}

// Takes a body-frame scan and world-frame pose, updates the occupancy grid
void Mapper::update_map(Scan& scan, Pose& pose) {
	Pose2D p;
	p.x_ = pose[0];
	p.y_ = pose[1];
	p.theta_ = pose[2];
	Scan world_scan = matcher.transformScanToPose(scan, p);
	grid.updateWithScan(world_scan, pose);
}

void Mapper::slam() {
	std::cout << "\nRunning pose graph optimization..." << std::endl;

	std::vector<Frame> frames = frame_history.snapshot();

	std::vector<Scan> scans;
	std::vector<Pose> poses;
	scans.reserve(frames.size());
	poses.reserve(frames.size());
	for (size_t i = 0; i < frames.size(); i++) {
		scans.push_back(frames[i].scan);
		poses.push_back(frames[i].pose);
	}

	int ind_interval = 2;
	auto result = mapping_optimized(scans, poses, posegraph, ind_interval);
	std::vector<Pose> corrected_poses = result.first;
	std::vector<int> nodes = result.second;

	std::cout << "Optimization complete! Optimized " << nodes.size() << " nodes" << std::endl;

	// Write corrected node poses back into frame_history
	std::vector<Pose> node_poses;
	node_poses.reserve(nodes.size());
	for (int k : nodes) node_poses.push_back(corrected_poses[k]);
	frame_history.update_poses(nodes, node_poses);

	// Rebuild occupancy grid from corrected node frames
	grid.clear();
	for (int k : nodes) {
		update_map(scans[k], corrected_poses[k]);
	}

	// Anchor curr_pose to the last corrected pose
	curr_pose.x_     = corrected_poses.back().x;
	curr_pose.y_     = corrected_poses.back().y;
	curr_pose.theta_ = corrected_poses.back().theta;

	grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5);
	grid.writePGMFile("occupancy_grid_slam.pgm");
}

bool Mapper::plan_path(const Pose2D& goal) {
	grid.inflateObstacles(0.1);
	this->path = grid.planPath(
		std::make_pair(curr_pose.x_, curr_pose.y_),
		std::make_pair(goal.x_, goal.y_)
	);
	for (std::pair<double,double> pt : path) {
		std::pair<int,int> p = grid.worldToGrid(pt.first, pt.second);
		grid.setLogOdds(p.first, p.second, -10.0);
	}
	return true;
}
