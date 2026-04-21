#ifndef FRAMEHISTORY_H
#define FRAMEHISTORY_H
#include <stdlib.h>
#include <vector>
#include <mutex>
#include "../include/scan_match_11.h"
#include "../include/pose.h"
 

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
#endif

