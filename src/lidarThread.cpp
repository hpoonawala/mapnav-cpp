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
#include "../include/lidarThread.h"


void LidarThread::run(){
    Scan scan_curr;
	int n_samples=0;
    VectorXi quality(8152);
	while(!shutdown_){
		auto result = scanner.capture(scan_curr, n_samples, quality);
        if (result == SL_RESULT_OPERATION_TIMEOUT) continue;
		std::lock_guard<std::mutex> lock(shared_.mtx);
        shared_.scan = scan_curr;
        shared_.fresh = true;
	}
}

LidarThread::LidarThread(const char *p1, sl_u32 p2) : 
	scanner(p1,p2), shutdown_(false)
{

	if(!scanner.initialize())
		throw std::runtime_error("LidarScanner init failed"); // If the initialization fails, return
	scanner.startScanning();
	thread_=std::thread(&LidarThread::run,this);
};

LidarThread::~LidarThread(){
	shutdown_ = true;
	thread_.join();
}

bool LidarThread::getScan(Scan& out){
	std::lock_guard<std::mutex> lock(shared_.mtx);
	if (!shared_.fresh) return false; // already got this one
	std::swap(out,shared_.scan);
	shared_.fresh = false;
	return true; // 
}

