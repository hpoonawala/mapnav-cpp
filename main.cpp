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
#include "OccupancyGrid.h"
#include "slam_posegraph.h"


using namespace sl;
using namespace std;
using namespace Eigen;

typedef MatrixXd Scan;  // Nx2 matrix where each row is (x,y)

void print_usage(int argc, const char * argv[])
{
	printf("Simple LIDAR data grabber for SLAMTEC LIDAR.\n"
			"Version:  %s \n"
			"Usage:\n"
			" For serial channel %s --channel --serial <com port> [baudrate]\n"
			" The baudrate used by different models is as follows:\n"
			"  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
			"A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
			" For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
			"The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
			, SL_LIDAR_SDK_VERSION,  argv[0], argv[0]);
}


void plot_histogram(sl_lidar_response_measurement_node_hq_t * nodes, size_t count)
{
	const int BARCOUNT =  75;
	const int MAXBARHEIGHT = 20;
	// const float ANGLESCALE = 360.0f/BARCOUNT;

	float histogram[BARCOUNT];
	for (int pos = 0; pos < _countof(histogram); ++pos) {
		histogram[pos] = 0.0f;
	}

	float max_val = 0;
	for (int pos =0 ; pos < (int)count; ++pos) {
		int int_deg = (int)(nodes[pos].angle_z_q14 * 90.f / 16384.f);
		if (int_deg >= BARCOUNT) int_deg = 0;
		float cachedd = histogram[int_deg];
		if (cachedd == 0.0f ) {
			cachedd = nodes[pos].dist_mm_q2/4.0f;
		} else {
			cachedd = (nodes[pos].dist_mm_q2/4.0f + cachedd)/2.0f;
		}

		if (cachedd > max_val) max_val = cachedd;
		histogram[int_deg] = cachedd;
	}

	for (int height = 0; height < MAXBARHEIGHT; ++height) {
		float threshold_h = (MAXBARHEIGHT - height - 1) * (max_val/MAXBARHEIGHT);
		for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
			if (histogram[xpos] >= threshold_h) {
				putc('*', stdout);
			}else {
				putc(' ', stdout);
			}
		}
		printf("\n");
	}
	for (int xpos = 0; xpos < BARCOUNT; ++xpos) {
		putc('-', stdout);
	}
	printf("\n");
}

// Capture and display is responsible for processing the data from LiDAR and putting it into scan as accepted Cartesian points. 
sl_result capture_and_display(ILidarDriver * drv, Scan& scan, int& n_samples, VectorXi& quality)
{
	sl_result ans;

	sl_lidar_response_measurement_node_hq_t nodes[8192];//array.
	size_t   count = _countof(nodes);
	// once we have count we can define Scan


	ans = drv->grabScanDataHq(nodes, count, 0);
	printf("\n %lu waiting for data...\n",count);
	int local_count = 0;
	double dist;
	double angle;
	double last_angle = -999.0; // Initialize to impossible value
	double angle_threshold = 0.5 * M_PI / 180.0; // 1 degree in radians

	if (SL_IS_OK(ans) || ans == SL_RESULT_OPERATION_TIMEOUT) {
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos) {
			/* printf("%s theta: %03.2f Dist: %08.2f  Q: %d \n", */ 
			/* 		(nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", */ 
			/* 		(nodes[pos].angle_z_q14 * 90.f) / 16384.f, */
			/* 		nodes[pos].dist_mm_q2/4.0f, */
			/* 		nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT); */
			if ( ( (int)(nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) ) > 40){
				angle = (nodes[pos].angle_z_q14 * 1.5708) / 16384.f;





				dist = nodes[pos].dist_mm_q2/4000.0f;

				scan(local_count,0) = dist*cos(angle);
				scan(local_count,1) = dist*sin(angle);
				quality(local_count) = (int) (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

				local_count++;
			}
			/* printf(" theta: %03.2f Dist: %08.2f \n", scan(pos,0),scan(pos,1)); */
		}
		n_samples = local_count;
		printf(" local count: %d",local_count);
	} else {
		printf("error code: %x\n", ans);
	}


	return ans;
}

int main(int argc, const char * argv[]) {
	const char *opt_channel = NULL;
	const char *opt_channel_param_first = NULL;
	sl_u32      opt_channel_param_second = 0;
	sl_result   op_result;
	int         opt_channel_type = CHANNEL_TYPE_SERIALPORT;

	IChannel* _channel;

	if (argc < 5) {
		print_usage(argc, argv);
		return -1;
	}

	const char * opt_is_channel = argv[1];
	if(strcmp(opt_is_channel, "--channel")==0)
	{
		opt_channel = argv[2];
		if(strcmp(opt_channel, "-s")==0||strcmp(opt_channel, "--serial")==0)
		{
			opt_channel_param_first = argv[3];
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
		}
		else if(strcmp(opt_channel, "-u")==0||strcmp(opt_channel, "--udp")==0)
		{
			opt_channel_param_first = argv[3];
			if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
			opt_channel_type = CHANNEL_TYPE_UDP;
		}
		else
		{
			print_usage(argc, argv);
			return -1;
		}
	}
	else
	{
		print_usage(argc, argv);
		return -1;
	}

	// create the driver instance
	ILidarDriver * drv = *createLidarDriver();

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	sl_lidar_response_device_health_t healthinfo;
	sl_lidar_response_device_info_t devinfo;
	do {
		// try to connect
		if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
			_channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
		}
		else if (opt_channel_type == CHANNEL_TYPE_UDP) {
			_channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
		}

		if (SL_IS_FAIL((drv)->connect(_channel))) {
			switch (opt_channel_type) {	
				case CHANNEL_TYPE_SERIALPORT:
					fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
							, opt_channel_param_first);
					break;
				case CHANNEL_TYPE_UDP:
					fprintf(stderr, "Error, cannot connect to the ip addr %s with the udp port %u.\n"
							, opt_channel_param_first, opt_channel_param_second);
					break;
			}
		}

		// retrieving the device info
		////////////////////////////////////////
		op_result = drv->getDeviceInfo(devinfo);

		if (SL_IS_FAIL(op_result)) {
			if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
				// you can check the detailed failure reason
				fprintf(stderr, "Error, operation time out.\n");
			} else {
				fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
				// other unexpected result
			}
			break;
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
			break;
		}


		if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want slamtec lidar to be reboot by software
			// drv->reset();
			break;
		}

		switch (opt_channel_type) 
		{	
			case CHANNEL_TYPE_SERIALPORT:
				drv->setMotorSpeed();
				break;
		}

		Scan scan_curr_loader(8152,3);  // 100 points
		Scan scan_curr;  // 100 points
		VectorXi quality(8152);  // 100 points

		int n_samples=0;


		// take only one 360 deg scan and display the result as a histogram
		////////////////////////////////////////////////////////////////////////////////
		if (SL_IS_FAIL(drv->startScan( 0,1 ))) // you can force slamtec lidar to perform scan operation regardless whether the motor is rotating
		{
			fprintf(stderr, "Error, cannot start the scan operation.\n");
			break;
		}

		delay(1500);  // This delay seems to just start a scan mode. Why is there a 3000ms delay? Seems like you don't get anything at 300,1000 ms. Got at 1500,2000

		std::string data_target = "localdata_2.txt";
		// define output file and clear it
		std::ofstream scan_file;
		scan_file.open(data_target); // clears file 
		scan_file.close();
		scan_file.open(data_target,std::ios::app );

		// This does it once, then does the histogram and display. D
		int loop_iters = 100;
		for(int k=0;k<loop_iters;k++){
			printf("Loop: %d / %d\n",k,loop_iters); 
			auto start = std::chrono::high_resolution_clock::now();
			if (SL_IS_FAIL(capture_and_display(drv,scan_curr_loader,n_samples,quality))) {
				fprintf(stderr, "Error, cannot grab scan data.\n");
				break;
			} else {
				//We have updates, get a scan match
				scan_curr = scan_curr_loader.topRows(n_samples).leftCols(2);
				for(int ii = 0; ii < n_samples; ii++){
					scan_file << scan_curr(ii,0) << "," << scan_curr(ii,1) << ",";
				}
				scan_file << "\n";

			}
			auto end = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
			const int threshold_ms = 150;

			if (duration.count() < threshold_ms) {
				int needed_delay = threshold_ms - duration.count();
				delay(needed_delay);
			}
			delay(150); // Seems to be how long it takes to collect the next scan
						// Scan matching goes here
			printf("NM Time elapsed: "); printf("%d",duration.count());printf(" milliseconds.\n");
		}

		scan_file.close();
	} while(0); // do once loop. why?

	drv->stop();
	switch (opt_channel_type) 
	{	
		case CHANNEL_TYPE_SERIALPORT:
			delay(20);
			drv->setMotorSpeed(0);
			break;
	}
	if(drv) {
		delete drv;
		drv = NULL;
	}
	return 0;
}
