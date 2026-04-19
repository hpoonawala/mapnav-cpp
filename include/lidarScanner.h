#ifndef LIDAR_H
#define LIDAR_H 

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
using namespace sl;
using namespace std;
using namespace Eigen;
typedef MatrixXd Scan;  // Nx2 matrix where each row is (x,y)
class LidarScanner{
	private:
		IChannel* _channel;
		Scan scan_curr_loader;  // 100 points
		ILidarDriver *drv;
		// Disable copy constructor and assignment
		LidarScanner(const LidarScanner&) = delete;
		LidarScanner& operator=(const LidarScanner&) = delete;
	public:
		LidarScanner(const char*, sl_u32);
		~LidarScanner();
		void startScanning();
		void stopScanning();
		sl_result capture(Scan&, int&, VectorXi&);
		bool initialize();

};
#endif

