
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

// Slam, map, control includes
#include "OccupancyGrid.h"
#include "slam_posegraph.h"
#include "mapper.h"
#include "DDRCappController.h"

// Serial includes
#include "SerialWriter.h"
#include <iomanip>
#include <sstream>
#include <exception>

using namespace sl;
using namespace std;
using namespace Eigen;

typedef MatrixXd Scan;  // Nx2 matrix where each row is (x,y)

// Begin definitions 


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
		write_serial_message("/dev/ttyACM0", result.c_str());
		// if (toggle_state == 0) write_serial_message("/dev/ttyACM0", result.c_str());
		// else write_serial_message("/dev/ttyACM0", "S.+100.+100.0\n");



	} catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}

// LiDAR Scanner class 
class LidarScanner{
	private:
		IChannel* _channel;
		Scan scan_curr_loader;  // 100 points
		ILidarDriver *drv;
		// Disable copy constructor and assignment
		LidarScanner(const LidarScanner&) = delete;
		LidarScanner& operator=(const LidarScanner&) = delete;
	public:
		// Constructor 
		LidarScanner(const char *opt_channel_param_first, sl_u32 opt_channel_param_second) : drv {*createLidarDriver()}, _channel {(*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second))}, scan_curr_loader {Scan(8152,2)}  {};
		// Destructor
		~LidarScanner() { 
			delay(20);
			drv->setMotorSpeed(0);
			delete drv;
			drv = NULL;
		}

		void startScanning(){
			if (SL_IS_FAIL(drv->startScan( 0,1 ))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
			{
				fprintf(stderr, "Error, cannot start the scan operation.\n");
			}
			delay(1500);  // This delay seems to just start a scan mode. Why is there a 3000ms delay? Seems like you don't get anything at 300,1000 ms. Got at 1500,2000
		}
		void stopScanning(){
			drv->stop();
		}

		// getLidarScan function:
		sl_result capture(Scan& scan, int& n_samples, VectorXi& quality)
		{
			sl_result ans;

			sl_lidar_response_measurement_node_hq_t nodes[8192];//array.
			size_t   count = _countof(nodes);
			// once we have count we can define Scan

			//printf("\n %lu waiting for data...\n",count);
			ans = drv->grabScanDataHq(nodes, count, 200);
			//printf("initial error code: %x\n", ans);

			int local_count = 0;
			double dist;
			double angle;
			double last_angle = -999.0; // Initialize to impossible value
			double angle_threshold = 1.0 * M_PI / 180.0; // 1 degree in radians

			if (SL_IS_OK(ans)) {
				drv->ascendScanData(nodes, count);
				for (int pos = 0; pos < (int)count ; ++pos) {
					if ( ( (int)(nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) ) > 40){
						angle = (nodes[pos].angle_z_q14 * 1.5708) / 16384.f;
						dist = nodes[pos].dist_mm_q2/4000.0f;
						scan_curr_loader(local_count,0) = angle;
						scan_curr_loader(local_count,1) = dist;
						quality(local_count) = (int) (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
						local_count++;
					}
					/* printf(" theta: %03.2f Dist: %08.2f \n", scan(pos,0),scan(pos,1)); */
				}
				n_samples = local_count;
				//printf("local count: %d\n",local_count);
				scan = scan_curr_loader.topRows(n_samples);
			} else if (ans == SL_RESULT_OPERATION_TIMEOUT) {
			} else {
				printf("error code: %x\n", ans);
				fprintf(stderr, "Error, cannot grab scan data.\n");
			}

			return ans;
		}

		bool initialize(){
			if (!drv) {
				fprintf(stderr, "insufficent memory, exit\n");
				exit(-2);
			}
			// try to connect
			if (SL_IS_FAIL((drv)->connect(_channel))) {
				fprintf(stderr, "Error, cannot bind to the specified serial port.\n");
			} else printf("connected");
			// retrieving the device info
			////////////////////////////////////////
			sl_result   op_result;
			sl_lidar_response_device_info_t devinfo;
			op_result = drv->getDeviceInfo(devinfo);

			if (SL_IS_FAIL(op_result)) {
				if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
					// you can check the detailed failure reason
					fprintf(stderr, "Error, operation time out.\n");
				} else {
					fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
					// other unexpected result
				}
				return false; // break only made sense in the do-while loop
			}
			// print out the device serial number, firmware and hardware version number..
			printf("SLAMTEC LIDAR S/N: ");
			for (int pos = 0; pos < 16 ;++pos) {
				printf("%02X", devinfo.serialnum[pos]);
			}

			printf("\n"
					"Version:  %s \n"
					"Firmware Ver: %d.%02d\n"
					"Hardware Rev: %d\n"
					, "SL_LIDAR_SDK_VERSION"
					, devinfo.firmware_version>>8
					, devinfo.firmware_version & 0xFF
					, (int)devinfo.hardware_version);
			// check the device health
			////////////////////////////////////////
			sl_lidar_response_device_health_t healthinfo;
			op_result = drv->getHealth(healthinfo);
			if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
				printf("Lidar health status : ");
				switch (healthinfo.status) 
				{
					case SL_LIDAR_STATUS_OK:
						printf("OK.");
						break;
					case SL_LIDAR_STATUS_WARNING:
						printf("Warning.");
						break;
					case SL_LIDAR_STATUS_ERROR:
						printf("Error.");
						break;
				}
				printf(" (errorcode: %d)\n", healthinfo.error_code);

			} else {
				fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
				return false;
			}
			if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
				fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
				// enable the following code if you want slamtec lidar to be reboot by software
				// drv->reset();
				return false;
			}

			drv->setMotorSpeed();
			return true;

		} // End initialize
};


// Capture and display is responsible for processing the data from LiDAR and putting it into scan as accepted/filtered Cartesian points. 
// Main program
int main(int argc, const char * argv[]) {
	const char *opt_channel = NULL;
	const char *opt_channel_param_first = NULL;
	sl_u32      opt_channel_param_second = 0;
	sl_result   op_result;
	int         opt_channel_type = CHANNEL_TYPE_SERIALPORT;


	if (argc < 5) { // cmd --channel --serial port baud_rate
		printf("Incorrect usage. ./lidar.exe --channel --serial <port> 115200");
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

	std::string data_target = "localdata_2.txt";
	// define output file and clear it
	std::ofstream scan_file;
	scan_file.open(data_target); // clears file 
	scan_file.close();
	scan_file.open(data_target,std::ios::app );

	// Start loop
	int loop_iters = 150;
	sl_result lidar_result;
	Mapper mapper;
	DDRCappController controller({}, 0.5, 100.0, 60.0);
	auto start = std::chrono::high_resolution_clock::now();
	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	for(int k=0;k<loop_iters;k++){
		printf("Loop: %d / %d\n",k,loop_iters); 
		lidar_result = lidarScanner.capture(scan_curr,n_samples,quality); // fills scan_curr with polar version
		if (lidar_result == SL_RESULT_OPERATION_TIMEOUT) continue;
		mapper.update_scans(scan_curr);
		std::cout << mapper.curr_pose << "\n";
		for(int ii = 0; ii < n_samples; ii++){
			scan_file << scan_curr(ii,0) << "," << scan_curr(ii,1) << "," << quality(ii) << ",";
		}scan_file << "\n";
		if (k % 10 == 0 && k > 5) {
			send_stop();
			mapper.slam();
			mapper.plan_path(Pose2D(2.3,-0.5,0.0));
			if (!mapper.path.empty()) {
				controller.setPath(mapper.path);
			}

			mapper.grid.writePGMFile("occupancy_grid_slam_" + std::to_string(k) + ".pgm");

		}
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		printf("initial NM Time elapsed: "); printf("%d",duration.count());printf(" milliseconds.\n");
		start = std::chrono::high_resolution_clock::now();
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

	// Run slam 
	mapper.slam();
	mapper.plan_path(Pose2D(2.3,-0.5,0.0));
    //mapper.grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5);
    mapper.grid.writePGMFile("occupancy_grid_slam_final.pgm");
	return 0;
}
