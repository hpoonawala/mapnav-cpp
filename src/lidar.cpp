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

// Slam, map, control includes
#include "OccupancyGrid.h"
#include "slam_posegraph.h"
#include "mapper.h"
#include "DDRCappController.h"
#include "lidarScanner.h"
#include "lidarThread.h"

// Serial includes
#include "SerialWriter.h"
#include "MotorController.h"
#include <iomanip>
#include <sstream>
#include <exception>

static const char* MOTOR_PORT = "/dev/ttyACM0"; // Connection to ESP32s3
static const char* SCAN_DATA_FILE = "localdata_99.txt"; // where to store all the scans obtained

// Telemetry includes
#include "TelemetryServer.h"
#include "timer.h"

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

class ScanSaver {
	public:
		ScanSaver(const char* data_target) {
			scan_file.open(data_target, std::ios::out | std::ios::trunc); 
		}
		void save(const Scan& scan){
			for(int ii = 0; ii < scan.rows(); ii++){
				scan_file << scan(ii,0) << "," << scan(ii,1) << "," << 0.0 << ",";
			}
			scan_file << "\n";
		}
	private:
		std::ofstream scan_file;

};


// Capture and display is responsible for processing the data from LiDAR and putting it into scan as accepted/filtered Cartesian points. 
// Main program:
// Set up lidar, mapping objects, helper objects, 
// then loop of 
//     get-scan, 
//     pose odometry, 
//     occasional stop-for-SLAM correction
int main(int argc, const char * argv[]) {


	// Parse arguments necessary for starting the lidar. 
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
	const char * opt_is_channel = argv[1]; // --channel 
	opt_channel = argv[2]; // --serial
	opt_channel_param_first = argv[3]; // USB Port
	if (argc>4) opt_channel_param_second = strtoul(argv[4], NULL, 10); // gets baud rate
	sl_u32 num_steps;
	if (argc > 5) num_steps = strtoul(argv[5], NULL, 10);
	else num_steps=200;

	// Initialize the LiDAR
	// This next line can throw an exception that is not being handled anywhere for now
	LidarThread lidar(opt_channel_param_first,opt_channel_param_second);
	Scan scan_curr;  // 100 points, range scan
	ScanSaver scan_saver(SCAN_DATA_FILE);
	SerialWriter serial(MOTOR_PORT);

	// Start telemetry server
	TelemetryServer telemetry(8765);
	telemetry.start();
	Pose2D navigation_goal(1.0,  0.0, 0.0);
	bool navigation_paused = false;

	// Start loop
	int loop_iters = int(num_steps);
	sl_result lidar_result;
	Mapper mapper;	// stores scan history and manages map
	MotorController motor(MOTOR_PORT); // for sending commands to the motors 
	DDRCappController controller({}, 0.5, 100.0, 60.0); // compute control from pose and scan
	Timer timer; // to time events. .reset() and .mark(string)
	Timer slam_timer; // to time events. .reset() and .mark(string)
	int n_frames =0;
	double last_heading = 0.0;
	bool have_heading = false;
	for(int k=0;k<loop_iters;k++){
		n_frames = mapper.frame_history.size();

		// Get Scan 
		if (!lidar.getScan(scan_curr)) {  // no new scan
			/* cout <<"delay 10ms\n"; */ 
			delay(10); 
			continue;
		} // but will not run SLAM, could lead to a very delayed SLAM update if lidar doesn't return for a while despite moving

		printf("Loop: %d / %d scans: %d\n",k,loop_iters,n_frames); 
		// 2. Request IMU heading from ESP32S3 and compute delta
		double dtheta_hint = 0.0;
		ImuReading imu = request_imu(serial);
		if (imu.valid) {
			if (have_heading)
				std::cout << "imu: " << imu.heading << "\n";
				dtheta_hint = Pose2D::normalizeAngle(imu.heading - last_heading);
			last_heading = imu.heading;
			have_heading = true;
		}
		mapper.update_scans(scan_curr,dtheta_hint);
		scan_saver.save(scan_curr);
		std::cout << "Current pose according to mapper:" << mapper.curr_pose << "\n";
		timer.mark("initial NM Time elapsed: "); timer.reset();
		// Publish pose to telemetry clients occasionally
		if (n_frames % 5 ==0) {
			telemetry.publishPose(mapper.curr_pose, k);
		}
		telemetry.tick(); // process telemetry

		// Process telemetry commands. Typically fast for low command rate
		while (telemetry.hasCommand()) {
			TelemetryCommand cmd = telemetry.popCommand();
			if (cmd.type == CommandType::SET_GOAL) {
				navigation_goal = Pose2D(cmd.x, cmd.y, 0.0);
				//mapper.plan_path(navigation_goal);
				//telemetry.publishPath(mapper.path);
				navigation_paused = false;
				std::cout << "Telemetry: New goal set to (" << cmd.x << ", " << cmd.y << ")\n";
			} else if (cmd.type == CommandType::STOP) {
				navigation_paused = true;
				motor.stop();
				std::cout << "Telemetry: Navigation paused\n";
			} else if (cmd.type == CommandType::RESUME) {
				navigation_paused = false;
				std::cout << "Telemetry: Navigation resumed\n";
			} else if (cmd.type == CommandType::REQUEST_MAP) {
				telemetry.publishMap(mapper.grid);
				//telemetry.publishPath(mapper.path);
				telemetry.tick();
			}
		}

		// try to see if SLAM is done:
		// SLAM also updates grid and calculates a new path
		bool slam_collect_res = mapper.slam_thread.tryCollect(mapper.frame_history, mapper.grid, mapper.curr_pose,mapper.path);
		cout << "slam_collect_res: " << slam_collect_res << "\n";
		if (slam_collect_res){
			slam_timer.mark("SLAM time elapsed: ");
			if (!mapper.path.empty()) {
				controller.setPath(mapper.path);
			}

			mapper.grid.writePGMFile("occupancy_grid_slam_" + std::to_string(n_frames) + ".pgm");

			// Publish map and path to telemetry clients
			telemetry.publishMap(mapper.grid);
			telemetry.tick();
			/* grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5); */
			/* grid.writePGMFile("occupancy_grid_slam.pgm"); */
			telemetry.publishPath(mapper.path);
		}
		// Check for SLAM point
		if (n_frames % 10 == 0 && n_frames > 5) {
			bool slam_launch_res = mapper.slam_thread.tryLaunch(mapper.frame_history, navigation_goal);
			cout << "slam_launch_res: " << slam_launch_res << "\n";
			if(slam_launch_res) slam_timer.reset();
		}

		// Send out control based on current pose
		// No motion until we get one slam run
		if (n_frames > 10 && !navigation_paused) {
			std::pair<int,int> velocities = controller.computeControl(mapper.curr_pose, scan_curr);
			std::cout << "velocities:" << velocities.first << " " << velocities.second << "\n";
			motor.send(velocities.first,velocities.second);
		} else {
			motor.stop();
		}
	}

	motor.stop(); // Stop the wheels 

	// Stop telemetry server
	telemetry.stop();

	mapper.slam_thread.wait();


	// TODO: Run final slam 
	/* mapper.slam(); */
	/* mapper.plan_path(navigation_goal); */
	//mapper.grid.writeGridToFile("occupancy_grid_slam.txt", 3.0, 1.5);
	mapper.grid.writePGMFile("occupancy_grid_slam_final.pgm");
	return 0;
}
