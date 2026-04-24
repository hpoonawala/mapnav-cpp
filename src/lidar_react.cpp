
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
#include <time.h>
#include <atomic>

using namespace sl;
using namespace std;
using namespace Eigen;

struct ImuReading {
	double heading; // fused heading in radians from ESP32S3
	bool valid;
};

// Sends "I.\n" and parses response. ESP32S3 format: "I.+0045.32\n" (yaw in degrees, %+08.2f)
ImuReading request_imu(SerialWriter& serial) {
	try {
		std::string resp = serial.write_and_read("I.\n", 500);
		if (resp.size() > 2 && resp[0] == 'I' && resp[1] == '.') {
			double yaw_deg = std::stod(resp.substr(2));
			return {yaw_deg * M_PI / 180.0, true};
		}
	} catch (const std::exception& e) {
		std::cerr << "IMU request failed: " << e.what() << std::endl;
	}
	return {0.0, false};
}

void send_stop(SerialWriter& serial) {
	try {
		serial.write_and_read("S.+000.+000.0\n");
	} catch (const std::exception& e) {
		std::cerr << "send_stop error: " << e.what() << std::endl;
	}
}

void send_cmd(SerialWriter& serial, int v, int w) {
	bool signflag = w > 0 ? false : true;
	std::ostringstream oss_v, oss_w;
	oss_v << std::setfill('0') << std::setw(3) << std::abs(v);
	oss_w << std::setfill('0') << std::setw(3) << std::abs(w);
	std::string msg = "D.+";
	msg += oss_v.str();
	msg += ".";
	msg += (signflag ? "+" : "-");
	msg += oss_w.str();
	msg += ".0\n";
	try {
		serial.write_and_read(msg);
	} catch (const std::exception& e) {
		std::cerr << "send_cmd error: " << e.what() << std::endl;
	}
}

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

	const char * opt_is_channel = argv[1];
	opt_channel = argv[2];
	opt_channel_param_first = argv[3];
	if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);

	LidarScanner lidarScanner(opt_channel_param_first,opt_channel_param_second);
	if(lidarScanner.initialize()) ;
	else return -1;

	SerialWriter serial("/dev/ttyACM0");

	Scan scan_curr;
	VectorXi quality(8152);
	int n_samples=0;

	lidarScanner.startScanning();

	std::string data_target = "localdata_3.txt";
	std::ofstream scan_file;
	scan_file.open(data_target);
	scan_file.close();
	scan_file.open(data_target, std::ios::app);

	int loop_iters = 300;
	sl_result lidar_result;
	Mapper mapper;
	DDRCappController controller({}, 0.5, 200.0, 060.0);
	double last_heading = 0.0;
	bool have_heading = false;

	for(int k=0; k<loop_iters; k++){
		printf("Loop: %d / %d\n", k, loop_iters);

		// 1. Capture LIDAR scan
		lidar_result = lidarScanner.capture(scan_curr, n_samples, quality);
		if (lidar_result == SL_RESULT_OPERATION_TIMEOUT) continue;
		for(int ii = 0; ii < n_samples; ii++){
			scan_file << scan_curr(ii,0) << "," << scan_curr(ii,1) << "," << quality(ii) << ",";
		}
		scan_file << "\n";

		// 2. Request IMU heading from ESP32S3 and compute delta
		double dtheta_hint = 0.0;
		ImuReading imu = request_imu(serial);
		if (imu.valid) {
			if (have_heading)
				dtheta_hint = Pose2D::normalizeAngle(imu.heading - last_heading);
			last_heading = imu.heading;
			have_heading = true;
		}

		// 3. Scan match using IMU heading delta as initial guess
		if (n_samples > 0)
			mapper.update_scans(scan_curr, dtheta_hint);

		// 4. Compute control and send motor command
		if (k > 10) {
			std::pair<int,int> velocities = controller.computeControl(mapper.curr_pose, scan_curr);
			std::cout << "velocities:" << velocities.first << " " << velocities.second << "\n";
			send_cmd(serial, velocities.first, velocities.second);
		} else {
			send_stop(serial);
		}
	}
	scan_file.close();

	send_stop(serial);
	lidarScanner.stopScanning();

	return 0;
}
