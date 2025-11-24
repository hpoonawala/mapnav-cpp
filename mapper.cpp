#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "pose.h"
#include "scan_match_11.h"
#include "OccupancyGrid.h"
#include "slam_posegraph.h"
#include "mapper.h"
#include <chrono>


Mapper::Mapper() :	matcher {}, 
					posegraph {matcher,Pose(0.0,0.0,0.0),0.85}, 
					grid {OccupancyGrid(10.0,10.0,0.02)},  
					gridsize{0.85}, 
					curr_pose{Pose2D(0.0,0.0,0.0) }, 
					nscans{0} 
					{};

Scan polarToCartesian(Scan& scan,int n_samples) {
	Scan cart_scan= Scan(n_samples,2);
	for (int i = 0; i < n_samples ; i++){
		cart_scan(i,0) = scan(i,1)*cos(scan(i,0));
		cart_scan(i,1) = -scan(i,1)*sin(scan(i,0));
	}
	return cart_scan;
}

void Mapper::update_scans(Scan& scan_polar) {
	Scan scan = polarToCartesian(scan_polar,scan_polar.rows());
	Pose2D result;
	Matrix3d hessian;
	if (nscans == 0) { // no matching on first scan, just save 
		localScans.push_back(scan);
		worldScans.push_back(scan);
		world_poses.push_back(Pose  (curr_pose.x_, curr_pose.y_,curr_pose.theta_));
	} else { // match, update pose, then save
		matcher.ndtScanMatchHP(localScans[nscans-1], scan,gridsize, result, hessian,60,  1e-6, 0.0,  0.0, 0.0, false);
		move_pose_local(curr_pose,result); // update current pose
		world_poses.push_back(Pose  (curr_pose.x_, curr_pose.y_,curr_pose.theta_));
		localScans.push_back(scan);
		worldScans.push_back( matcher.transformScanToPose(scan,curr_pose));
		// update the grid
	}
	nscans++;
};

// Takes a Cartesian scan in body frame and the world-frame pose of robot
void Mapper::update_map(Scan& scan, Pose& pose){ // this should be internal? 
	Scan world_scan_curr;  // 100 points
	Pose2D curr_pose;
	curr_pose.x_ = pose[0];
	curr_pose.y_ = pose[1];
	curr_pose.theta_ = pose[2];
	world_scan_curr = matcher.transformScanToPose(scan,curr_pose);
	grid.updateWithScan(world_scan_curr, pose);

}

// Runs the pose graph optimization 
void Mapper::slam(){
    std::cout << "\nRunning pose graph optimization..." << std::endl;
	int ind_interval=2;
    auto result = mapping_optimized(localScans, world_poses, posegraph, ind_interval);
    world_poses = result.first;
    auto nodes = result.second;
    
    std::cout << "Optimization complete!" << std::endl;
	std::cout << "Optimized " << nodes.size() << " nodes" << std::endl;
	grid.clear();
	for(int k : nodes){
		update_map(localScans[k],world_poses[k]);
	}
	curr_pose.x_ = world_poses.back()[0];
	curr_pose.y_ = world_poses.back()[1];
	curr_pose.theta_ = world_poses.back()[2];
    grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5);
    grid.writePGMFile("occupancy_grid_slam.pgm");
}

bool Mapper::plan_path(const Pose2D& goal){
	// Do we need to error check about existence of map?
	// Or does the plan_path take care of it?
	grid.inflateObstacles(0.1);
	this->path = grid.planPath(std::make_pair(curr_pose.x_,curr_pose.y_),std::make_pair(goal.x_,goal.y_));
	for (std::pair<double, double> pt : path){
		std::pair<int,int> p = grid.worldToGrid(pt.first,pt.second);
		grid.setLogOdds(p.first,p.second,-10.0);
	}
	return true;
}
