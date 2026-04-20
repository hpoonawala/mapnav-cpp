#ifndef LIDAR_THREAD_H
#define LIDAR_THREAD_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

#include <thread>
#include <mutex>
#include <atomic>
#include "../include/lidarScanner.h"

#include "../include/scan_match_11.h"

struct SharedScan {
	std::mutex mtx;
	Scan scan;
	bool fresh = false;
};


class LidarThread{
	public: 
		LidarThread(const char *, sl_u32 );
		~LidarThread();
		bool getScan(Scan& ); // thread creator provides a place to put the scan
		// non-copyable
		LidarThread(const LidarThread&) = delete;
		LidarThread& operator=(const LidarThread&) = delete;
	private:
		SharedScan shared_;
		LidarScanner scanner;
		std::atomic<bool> shutdown_{false};
		std::thread thread_;
		void run();

};

#endif
