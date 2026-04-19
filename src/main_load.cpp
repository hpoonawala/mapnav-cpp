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
        
        // Extract x and y coordinates
        // x: values[0], values[2], values[4], ... (even indices)
        // y: values[1], values[3], values[5], ... (odd indices)
        int num_points = values.size() / 2;
        
        if (num_points == 0) {
            continue;
        }
        
        // Create Eigen matrix with shape (num_points, 2)
        Eigen::MatrixXd scan(num_points, 2);
        
        for (int i = 0; i < num_points; ++i) {
            scan(i, 0) = values[2 * i];      // x coordinate
            scan(i, 1) = -values[2 * i + 1]; // y coordinate (negated)
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
    std::string filename = "lidar_data.csv";
    
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
		printf("n scans: %d",loop_iters);
		for(int k=0;k<loop_iters;k++){
			printf("Loop: %d / %d\n",k,loop_iters); 
			//We have updates, get a scan match
			std::cout <<"\nrows: " << scans[k].rows() << "\n";
			mapper.update_scans(scans[k]);

						// Scan matching goes here
		}

		scan_file.close();
	} while(0); // do once loop. why?
				//
	for (Pose p : mapper.world_poses){
		std::cout << p[0] << " " << p[1] << " " << p[2] << "\n";
	}
				//
	// done with Odom, but we have no odom data

    std::cout << "\nRunning pose graph optimization..." << std::endl;
	int ind_interval=2;
    auto result = mapping_optimized(mapper.localScans, mapper.world_poses, mapper.posegraph, ind_interval);
    mapper.world_poses = result.first;
    auto nodes = result.second;
    
    std::cout << "Optimization complete!" << std::endl;
	std::cout << "Optimized " << nodes.size() << " nodes" << std::endl;
	Scan scan_curr;  // 100 points
	Scan world_scan_curr;  // 100 points
	int loop_iters = scans.size();
	Pose2D curr_pose;
	for(int k : nodes){
		scan_curr=mapper.localScans[k];
		curr_pose.x_ = mapper.world_poses[k][0];
		curr_pose.y_ = mapper.world_poses[k][1];
		curr_pose.theta_ = mapper.world_poses[k][2];
		world_scan_curr = mapper.matcher.transformScanToPose(scan_curr,curr_pose);
		mapper.grid.updateWithScan(world_scan_curr, mapper.world_poses[k]);
	}
    mapper.grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5);
    mapper.grid.writePGMFile("occupancy_grid_slam.pgm");
	return 0;
}
