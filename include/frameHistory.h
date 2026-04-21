#ifndef FRAMEHISTORY_H
#define FRAMEHISTORY_H
#include <stdlib.h>
#include <vector>
#include <mutex>
#include "../include/scan_match_11.h"
#include "../include/pose.h"
 
// We moved these scan transforms here because they combine scans and pose
// Scan match in place to avoid reallocations
inline void transformScanInPlace(Scan& output, const Scan& scan,
                                          double tx, double ty, double phi) {
    double c = cos(phi);
    double s = sin(phi);
    
    for (int i = 0; i < scan.rows(); ++i) {
        double x = scan(i, 0);
        double y = scan(i, 1);
        output(i, 0) = c*x - s*y + tx;
        output(i, 1) = s*x + c*y + ty;
    }
}

inline Scan transformScan(const Scan& scan, double tx, double ty, double phi) {
	Matrix2d rotation;
	rotation << cos(phi), -sin(phi),
			   sin(phi),  cos(phi);
	
	Vector2d translation(tx, ty);
	
	Scan transformed = (rotation * scan.transpose()).transpose();
	transformed.rowwise() += translation.transpose();
	
	return transformed;
}

inline Scan transformScanToPose(const Scan& scan, const Pose2D& pose) {
	return transformScan(scan, pose.x_, pose.y_, pose.theta_);
}

struct Frame {
	Scan scan;   // body-frame
	Pose2D pose;   // world-frame pose estimate at capture time
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

	void update_poses(const std::vector<int>& indices, const std::vector<Pose2D>& poses) {
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

