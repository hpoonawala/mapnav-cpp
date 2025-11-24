// TODO: upgrade lidar.cpp using this template to improve `lidarScanner`
#ifndef LIDAR_SCANNER_H
#define LIDAR_SCANNER_H

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <Eigen/Dense>
#include <string>

using namespace sl;
using namespace Eigen;

enum class LidarStatus {
    OK,
    WARNING,
    ERROR,
    NOT_INITIALIZED
};

struct ScanData {
    MatrixXd points;        // Nx2 matrix of (x,y) points
    VectorXi quality;       // Quality values for each point
    int n_samples;          // Number of valid samples
    double timestamp;       // Capture timestamp
};

class LidarScanner {
public:
    // Constructor/Destructor
    LidarScanner();
    ~LidarScanner();

    // Initialization
    bool initializeSerial(const char* port, sl_u32 baudrate);
    bool initializeUdp(const char* ipaddr, sl_u32 port);
    
    // Core functionality
    bool startScanning();
    bool stopScanning();
    bool getNextScan(ScanData& scan_data);
    
    // Status and health
    LidarStatus getStatus() const;
    bool isHealthy() const;
    void getDeviceInfo(std::string& info) const;
    
private:
    // Disable copy constructor and assignment
    LidarScanner(const LidarScanner&) = delete;
    LidarScanner& operator=(const LidarScanner&) = delete;
    
    // Internal helper methods
    bool connect(IChannel* channel);
    bool checkHealth();
    bool captureRawScan(sl_lidar_response_measurement_node_hq_t* nodes, size_t& count);
    void processRawScan(sl_lidar_response_measurement_node_hq_t* nodes, size_t count, ScanData& scan_data);
    void cleanup();
    
    // Member variables
    ILidarDriver* drv_;
    IChannel* channel_;
    bool initialized_;
    bool scanning_;
    LidarStatus status_;
    int quality_threshold_;
};

#endif // LIDAR_SCANNER_H
