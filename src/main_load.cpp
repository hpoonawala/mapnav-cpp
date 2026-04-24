/*
 *  SLAMTEC LIDAR
 *  Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// FROM `../rplidar_sdk/app/grabber/sim/`
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

// scan match includes
#include "scan_match_11.h"
#include "pose.h"
#include <cmath>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
	while (ms>=1000){
		usleep(1000*1000);
		ms-=1000;
	};
	if (ms!=0)
		usleep(ms*1000);
}
#endif

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "OccupancyGrid.h"
#include "slam_posegraph.h"
#include "mapper.h"

std::vector<Eigen::MatrixXd> loadLidarScans(const std::string& filename) {
    std::vector<Eigen::MatrixXd> scanlist;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return scanlist;
    }
    
    std::string line;
    int scan_count = 0;
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> values;
        
        // Parse all comma-separated values
        while (std::getline(ss, value, ',')) {
            try {
                values.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Warning: Could not parse value: " << value << std::endl;
            }
        }
        
        std::cout << "elements: " << values.size() << std::endl;
        
        // Each point is (angle_rad, dist_m, quality) — 3 values per point
        int num_points = values.size() / 3;

        if (num_points == 0) {
            continue;
        }

        // Convert polar to Cartesian using same convention as polarToCartesian in mapper.cpp
        Eigen::MatrixXd scan(num_points, 2);
        for (int i = 0; i < num_points; ++i) {
            double angle = values[3 * i];
            double dist  = values[3 * i + 1];
            scan(i, 0) =  dist * cos(angle);  // x
            scan(i, 1) = -dist * sin(angle);  // y
        }
        
        scanlist.push_back(scan);
        scan_count++;
        std::cout << "scans: " << scan_count << std::endl;
    }
    
    file.close();
    return scanlist;
}

using namespace sl;
using namespace std;
using namespace Eigen;

int main(int argc, const char * argv[]) {
    std::string filename = "data/local_data5.txt";
    
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::cout << "Loading LiDAR scans from: " << filename << std::endl;
    std::vector<Eigen::MatrixXd> scans = loadLidarScans(filename);
    
    std::cout << "\nTotal scans loaded: " << scans.size() << std::endl;
    
    // Example: Access and print first scan's dimensions
    if (!scans.empty()) {
        std::cout << "First scan dimensions: " << scans[0].rows() 
                  << " x " << scans[0].cols() << std::endl;
        std::cout << "First scan first point: [" << scans[0](0, 0) 
                  << ", " << scans[0](0, 1) << "]" << std::endl;
    }
	Pose2D navigation_goal(1.0,  0.0, 0.0);
	Mapper mapper;
    do {



		std::string data_target = "localdata_redundant.txt";
		// define output file and clear it
		std::ofstream scan_file;
		scan_file.open(data_target); // clears file 
		scan_file.close();
		scan_file.open(data_target,std::ios::app );

		// This does it once, then does the histogram and display. D
		int loop_iters = scans.size();
		if (argc > 2) loop_iters = strtoul(argv[2], NULL, 10);
		printf("n scans: %d",loop_iters);
		for(int k=0;k<loop_iters;k++){
			/* printf("Loop: %d / %d\n",k,loop_iters); */ 
			//We have updates, get a scan match
			/* std::cout <<"\nrows: " << scans[k].rows() << "\n"; */
			// update_scans applies polarToCartesian internally; scans from file are
			// already Cartesian, so we replicate the rest of update_scans directly.
			int n = mapper.frame_history.size();
			cout << "scan: " << n << std::endl;
			if (n == 0) {
				mapper.frame_history.append({scans[k], mapper.curr_pose,0.0});
			} else {
				double scan_match_score;
				Scan prev_scan = mapper.frame_history.last_scan();
				Pose2D delta;
				Eigen::Matrix3d hessian;
				mapper.matcher.ndtScanMatchHP(prev_scan, scans[k], mapper.gridsize, delta, scan_match_score,hessian, 60, 1e-6, 0.0, 0.0, 0.0, true);
				if (scan_match_score > 150.0 ) {
				cout << "main_load score high: " << scan_match_score << "\n";
				mapper.matcher.ndtScanMatchHP(prev_scan, scans[k], mapper.gridsize * 2.0, delta, scan_match_score, hessian, 200, 1e-6, 0.0, 0.0, 0.0, true);
				cout << "main_load score high: " << scan_match_score << "\n";
				mapper.matcher.ndtScanMatchHP(prev_scan, scans[k], mapper.gridsize , delta, scan_match_score, hessian, 200, 1e-6, delta[0], delta[1], delta[2], true);
				}
				cout << "main_load score: " << scan_match_score << "\n";
				move_pose_local(mapper.curr_pose, delta);
				mapper.frame_history.append({scans[k], mapper.curr_pose});
			}
			if (k % 10 == 0 && k > 5) {
				bool slam_launch_res = mapper.slam_thread.tryLaunch(mapper.frame_history, navigation_goal);
				cout << "slam_launch_res: " << slam_launch_res << "\n";
				bool slam_collect_res = false;
				while (!slam_collect_res) {
					slam_collect_res = mapper.slam_thread.tryCollect(mapper.frame_history, mapper.grid, mapper.curr_pose, mapper.path);
					if (!slam_collect_res) delay(50);
				}
			}

						// Scan matching goes here
		}

		scan_file.close();
	} while(0); // do once loop. why?
				//
				//
	bool slam_launch_res = mapper.slam_thread.tryLaunch(mapper.frame_history, navigation_goal);
	cout << "slam_launch_res: " << slam_launch_res << "\n";
	bool slam_collect_res = false;
	while (!slam_collect_res) {
		slam_collect_res = mapper.slam_thread.tryCollect(mapper.frame_history, mapper.grid, mapper.curr_pose, mapper.path);
		if (!slam_collect_res) delay(50);
	}
	cout << "slam_collect_res: " << slam_collect_res << "\n";
	mapper.grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5);
	mapper.grid.writePGMFile("occupancy_grid_slam.pgm");
	return 0;
}
