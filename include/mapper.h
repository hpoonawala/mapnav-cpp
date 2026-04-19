#ifndef _MAPPER_H
#define _MAPPER_H
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
Scan cartesianToPolar(Scan& ,int);
class Mapper {
	public: 
		NDTScanMatcher matcher;
		PoseGraph posegraph;
		OccupancyGrid grid;
		double gridsize;
		Pose2D curr_pose;
		vector<pair<double,double>> path;
		vector<Scan> localScans;
		int nscans;
		vector<Scan> worldScans;
		std::vector<Pose> world_poses;
		vector<int> updated_indices;
		int index_interval;
		//Constructor
		Mapper();
	
	void update_scans(Scan&);
	void update_map(Scan&, Pose&);
	bool plan_path(const Pose2D& goal);
	void slam();




};
#endif
