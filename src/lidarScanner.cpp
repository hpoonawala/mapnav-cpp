
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

// Note that the Scan returned has only n_samples in it
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

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

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include "../include/lidarScanner.h"

static const int LIDAR_RETURN_QUALITY_THRESHOLD = 40;
						//
using namespace sl;
using namespace std;
using namespace Eigen;
/* typedef MatrixXd Scan;  // Nx2 matrix where each row is (x,y) */

		LidarScanner::LidarScanner(const char *opt_channel_param_first, sl_u32 opt_channel_param_second) : drv {*createLidarDriver()}, _channel {(*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second))}, scan_curr_loader {Scan(8152,2)}  {};
		// Destructor
		LidarScanner::~LidarScanner() { 
			delay(20);
			drv->setMotorSpeed(0);
			delete drv;
			drv = NULL;
		}

		void LidarScanner::startScanning(){
			if (SL_IS_FAIL(drv->startScan( 0,1 ))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
			{
				fprintf(stderr, "Error, cannot start the scan operation.\n");
			}
			delay(1500);  // This delay seems to just start a scan mode. Why is there a 3000ms delay? Seems like you don't get anything at 300,1000 ms. Got at 1500,2000
		}
		void LidarScanner::stopScanning(){
			drv->stop();
		}

		// getLidarScan function:
		sl_result LidarScanner::capture(Scan& scan, int& n_samples, VectorXi& quality)
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
			double angle_threshold = 0.33* M_PI / 180.0; // 1 degree in radians

			if (SL_IS_OK(ans)) {
				drv->ascendScanData(nodes, count);
				for (int pos = 0; pos < (int)count ; ++pos) {
					if ( ( (int)(nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) ) > LIDAR_RETURN_QUALITY_THRESHOLD){
						angle = (nodes[pos].angle_z_q14 * 1.5708) / 16384.f;
						if (last_angle < 0 || fabs(angle - last_angle) >= angle_threshold) {
							dist = nodes[pos].dist_mm_q2/4000.0f;
							scan_curr_loader(local_count,0) = angle;
							scan_curr_loader(local_count,1) = dist;
							quality(local_count) = (int) (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
							last_angle = angle;
							local_count++;
						}

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

		bool LidarScanner::initialize(){
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

			drv->setMotorSpeed(0);
			return true;

		} // End initialize

