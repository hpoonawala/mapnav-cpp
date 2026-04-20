#ifndef _MAPPER_H
#define _MAPPER_H
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <mutex>
#include "pose.h"
#include "scan_match_11.h"
#include "OccupancyGrid.h"
#include "slam_posegraph.h"

Scan cartesianToPolar(Scan&, int);

struct Frame {
	Scan scan;   // body-frame
	Pose pose;   // world-frame pose estimate at capture time
};

struct FrameHistory {
	std::mutex mtx;
	std::vector<Frame> frames;

	void append(const Frame& f) {
		std::lock_guard<std::mutex> lock(mtx);
		frames.push_back(f);
	}

	std::vector<Frame> snapshot() {
		std::lock_guard<std::mutex> lock(mtx);
		return frames;
	}

	void update_poses(const std::vector<int>& indices, const std::vector<Pose>& poses) {
		std::lock_guard<std::mutex> lock(mtx);
		for (size_t i = 0; i < indices.size(); i++) {
			frames[indices[i]].pose = poses[i];
		}
	}

	int size() {
		std::lock_guard<std::mutex> lock(mtx);
		return (int)frames.size();
	}

	Scan last_scan() {
		std::lock_guard<std::mutex> lock(mtx);
		return frames.back().scan;
	}
};

class Mapper {
	public:
		NDTScanMatcher matcher;
		PoseGraph posegraph;
		OccupancyGrid grid;
		FrameHistory frame_history;
		double gridsize;
		Pose2D curr_pose;
		vector<pair<double,double>> path;

		Mapper();

		void update_scans(Scan&);
		void update_map(Scan&, Pose&);
		bool plan_path(const Pose2D& goal);
		void slam();
};
#endif
