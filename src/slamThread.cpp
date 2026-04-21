#include <stdio.h>
#include "../include/slamThread.h"

// When we create SlamThread, it creates a PoseGraph, which needs the first three arguments
// PoseGraph uses its the scan matcher but through a cached scan matching system
// Calling the optimize function needs the fourth one
SlamThread::SlamThread(NDTScanMatcher& matcher, Pose initial, double gridsize, int ind_interval) : posegraph_{matcher,initial,gridsize}, ind_interval_{ind_interval} {};

bool SlamThread::tryLaunch(FrameHistory& frame_history){
	// if SLAM is running, nothing to be done
	bool running = future_.valid() && future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;
	if (running) return false; // did not launch 

	// Grab snapshot
	snapshot_ = frame_history.snapshot();

	// Convert it into a sequence of scans and poses (possibly doing this to keep mapping_optimized unchanged despite moving to use Frame)
	future_ = std::async(std::launch::async, [this]() {
		std::vector<Scan> scans;
		std::vector<Pose> poses;
		scans.reserve(snapshot_.size());
		poses.reserve(snapshot_.size());
		for (size_t i = 0; i < snapshot_.size(); i++) {
			scans.push_back(snapshot_[i].scan);
			poses.push_back(snapshot_[i].pose);
		}

		auto result = posegraph_.optimize(scans, poses, ind_interval_);
		corrected_poses_ = result.first;
		nodes_ = result.second;
	});
	std::cout << "Done launching slam\n";
	return true; // launched with future
}


bool SlamThread::tryCollect(FrameHistory& frame_history){
	if(!future_.valid()) return false;
	if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return false;

	// Write corrected node poses back into frame_history
	future_.get();
	// Write corrected node poses back into frame_history
	std::vector<Pose> node_poses;
	node_poses.reserve(nodes_.size());
	for (int k : nodes_) node_poses.push_back(corrected_poses_[k]);
	frame_history.update_poses(nodes_, node_poses);
	return true;

}

void SlamThread::wait(){
	if(future_.valid()) future_.get();
}


SlamThread::~SlamThread(){
	if(future_.valid()) future_.get();
}
