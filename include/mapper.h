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
#include "frameHistory.h"
#include "slamThread.h"

Scan cartesianToPolar(Scan&, int);

class Mapper {
	public:
		NDTScanMatcher matcher;
		SlamThread slam_thread;
		OccupancyGrid grid;
		FrameHistory frame_history;
		double gridsize;
		Pose2D curr_pose;
		vector<pair<double,double>> path;

		Mapper();

		void update_scans(Scan&);
		void update_map(Scan&, Pose&);
		bool plan_path(const Pose2D& goal);
};
#endif
