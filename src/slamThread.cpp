#include <stdio.h>
#include "../include/slamThread.h"

// When we create SlamThread, it creates a PoseGraph, which needs the first three arguments
// PoseGraph uses its the scan matcher but through a cached scan matching system
// Calling the optimize function needs the fourth one
SlamThread::SlamThread(NDTScanMatcher& matcher, Pose2D initial, double gridsize, int ind_interval, OccupancyGrid og) : posegraph_{matcher,initial,gridsize}, ind_interval_{ind_interval}, new_grid_{og} {};

bool SlamThread::tryLaunch(FrameHistory& frame_history,  const Pose2D& goal){
	// if SLAM is running, nothing to be done
	bool running = future_.valid() && future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;
	if (running) return false; // did not launch 

	// Grab snapshot
	snapshot_ = frame_history.snapshot();

	future_ = std::async(std::launch::async, [this, goal]() {
			auto result = posegraph_.optimize(snapshot_, ind_interval_);
			corrected_poses_ = result.first;
			nodes_ = result.second;

			// 3. Rebuild grid from corrected node frames
			new_grid_.clear();
			for (int k : nodes_) {
			Pose2D p(corrected_poses_[k].x_, corrected_poses_[k].y_, corrected_poses_[k].theta_);
			Scan world_scan = transformScanToPose(snapshot_[k].scan, p);
			new_grid_.updateWithScan(world_scan, corrected_poses_[k]);
			}
			// 4. Plan path on new grid
			new_grid_.inflateObstacles(0.1);
			new_path_ = new_grid_.planPath(
					std::make_pair(corrected_poses_.back().x_, corrected_poses_.back().y_),
					std::make_pair(goal.x_, goal.y_)
					);
	});
	std::cout << "Done launching slam\n";
	return true; // launched with future
}


bool SlamThread::tryCollect(FrameHistory& frame_history, OccupancyGrid& grid, Pose2D& curr_pose, std::vector<std::pair<double,double>>& path){
	if(!future_.valid()) return false;
	if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return false;

	// Write corrected node poses back into frame_history
	future_.get();
	// Write corrected node poses back into frame_history
	std::vector<Pose2D> node_poses;
	node_poses.reserve(nodes_.size());
	for (int k : nodes_) node_poses.push_back(corrected_poses_[k]);
	frame_history.update_poses(nodes_, node_poses);


	std::swap(grid, new_grid_); // Is this unsafe?
	curr_pose.x_     = corrected_poses_.back().x_;
	curr_pose.y_     = corrected_poses_.back().y_;
	curr_pose.theta_ = corrected_poses_.back().theta_;
	path = new_path_;
	return true;
}

void SlamThread::wait(){
	if(future_.valid()) future_.get();
}


SlamThread::~SlamThread(){
	if(future_.valid()) future_.get();
}
