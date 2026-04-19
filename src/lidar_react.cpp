
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

// scan match includes
#include "../include/scan_match_11.h"
#include "../include/pose.h"
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

// Slam, map, control includes
#include "../include/OccupancyGrid.h"
#include "../include/slam_posegraph.h"
#include "../include/mapper.h"
#include "../include/DDRCappController.h"
#include "../include/lidarScanner.h"

// Serial includes
#include "../include/SerialWriter.h"
#include <iomanip>
#include <sstream>
#include <exception>

// Watchdog timer
#include <pthread.h>
#include <time.h>
#include <atomic>

using namespace sl;
using namespace std;
using namespace Eigen;

// Controller command for stopping
int send_stop(){
	try {
		write_serial_message("/dev/ttyACM0", "S.+000.+000.0\n");



	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}

// Controller command for velocities 
int send_cmd(int v, int w){
	bool signflag = w > 0 ? false : true;
	std::ostringstream oss_v;
	oss_v << std::setfill('0') << std::setw(3) << std::abs(v);
	std::ostringstream oss;
	oss << std::setfill('0') << std::setw(3) << std::abs(w);
	// Build string now "D.sXXX.sXXX.0"
	std::string result = "D.+";
	result +=oss_v.str();  // 
	result +=".";  // "
	if (signflag) result +="+"; 
	else result+="-";
	result +=oss.str();  // "005"
	result +=".0\n";  // "005"
	//printf("\nw_raw: %f\n",w_raw);

	try {
		// Method 1: One-off message (closest to Python example)
		//std::cout << "=== Sending single command ===" << std::endl;
		//write_serial_message("/dev/ttyACM0", result.c_str());
		write_and_read_serial_message("/dev/ttyACM0", result.c_str(),115200,1000);
		// if (toggle_state == 0) write_serial_message("/dev/ttyACM0", result.c_str());
		// else write_serial_message("/dev/ttyACM0", "S.+100.+100.0\n");



	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}

// Capture and display is responsible for processing the data from LiDAR and putting it into scan as accepted/filtered Cartesian points. 
// Main program
int main(int argc, const char * argv[]) {
	const char *opt_channel = NULL;
	const char *opt_channel_param_first = NULL;
	sl_u32      opt_channel_param_second = 0;
	sl_result   op_result;
	int         opt_channel_type = CHANNEL_TYPE_SERIALPORT;


	if (argc < 5) { // cmd --channel --serial port baud_rate
		printf("Incorrect usage. Use: ./lidar.exe --channel --serial <port> 115200");
		return -1;
	}

	// Hard codes assumption that we are using SERIAL. Don't need argv[1] or 2], really.
	const char * opt_is_channel = argv[1];
	opt_channel = argv[2];
	opt_channel_param_first = argv[3];
	if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);

	// create the driver instance. Could create first thing, since it is independent of channel

	LidarScanner lidarScanner(opt_channel_param_first,opt_channel_param_second);
	if(lidarScanner.initialize()) ; // If the initialization fails, return with -1/ 
	else return -1;

	Scan scan_curr;  // 100 points, range scan
	VectorXi quality(8152);  // 100 points
	int n_samples=0;

	lidarScanner.startScanning();

	std::string data_target = "localdata_3.txt";
	// define output file and clear it
	std::ofstream scan_file;
	scan_file.open(data_target); // clears file 
	scan_file.close();
	scan_file.open(data_target,std::ios::app );

	// Start loop
	int loop_iters = 300;
	sl_result lidar_result;
	Mapper mapper;
	DDRCappController controller({}, 0.5, 200.0, 060.0);
	for(int k=0;k<loop_iters;k++){
		printf("Loop: %d / %d\n",k,loop_iters); 
		lidar_result = lidarScanner.capture(scan_curr,n_samples,quality); // fills scan_curr with polar version
		if (lidar_result == SL_RESULT_OPERATION_TIMEOUT) continue;
		for(int ii = 0; ii < n_samples; ii++){
			scan_file << scan_curr(ii,0) << "," << scan_curr(ii,1) << "," << quality(ii) << ",";
		}scan_file << "\n";
		if (k > 10) {
			std::pair<int,int> velocities = controller.computeControl(mapper.curr_pose, scan_curr);
			std::cout << "velocities:" << velocities.first << " " << velocities.second << "\n";
			send_cmd(velocities.first,velocities.second);
		} else {
			send_stop();
		}
	}
	scan_file.close();

	send_stop();

	lidarScanner.stopScanning();

	return 0;
}
