#include "Lidar_MRPT.h"

#include <algorithm>
#include <array>
#include <map>
#include <unistd.h>
#include <cctype>

#define NUM_SCANS 5


// Helper functions
namespace {
    // return True on success, false on failure
    bool setup_lidar_settings(CYdLidar& laser) {
        std::map<std::string, std::string> ports = ydlidar::lidarPortList();
        std::string port;
        
        // use auto detect and crash out if it fails
        if (ports.size() == 1) {
          port = ports.begin()->second;
        } else {
          // print size in case multiple 
          printf("List of lidar ports was of size: %d, exiting.\n", ports.size()); 
          return false;
        }
        
        // from datasheet
        int baudrate = 115200;
        bool isSingleChannel = true;
        float frequency = 7.0; // ?? 8.0;
        
        if (!ydlidar::os_isOk()) {
          return false;
        }
        
        //////////////////////string property/////////////////
        /// lidar usb port
        laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
        /// ignore array
        std::string ignore_array;
        ignore_array.clear();
        laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                          ignore_array.size());
        
        //////////////////////int property/////////////////
        /// lidar baudrate
        laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
        /// tof lidar
        int optval = TYPE_TRIANGLE;
        laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
        /// device type
        optval = YDLIDAR_TYPE_SERIAL;
        laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
        /// sample rate
        optval = 3;
        laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
        /// abnormal count
        optval = 4;
        laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
        
        //////////////////////bool property/////////////////
        /// fixed angle resolution
        bool b_optvalue = true; // TODO CHECK THIS AGAIN
        laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
        b_optvalue = false; // TODO CHECK THIS AGAIN
        /// rotate 180
        laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
        /// Counterclockwise
        laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
        b_optvalue = true;
        laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
        /// one-way communication
        laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
        /// intensity
        b_optvalue = false;
        laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
        /// Motor DTR
        b_optvalue = true;
        laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
        /// HeartBeat
        b_optvalue = false;
        laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));
        
        //////////////////////float property/////////////////
        /// unit: deg
        float f_optvalue = 180.0f;
        laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
        f_optvalue = -180.0f;
        laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
        /// unit: m
        f_optvalue = 8.f;
        laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = 0.10f;
        laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
        /// unit: Hz
        laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
        
        return true;
    }

    mrpt::obs::CObservation2DRangeScan getMRPTRangeScanFromScanData(const std::vector<LaserScan>& rawScans) {
        mrpt::obs::CObservation2DRangeScan resultScan;
	size_t scanSize = 0;
	for (const auto& rawScan : rawScans) scanSize += rawScan.points.size();
        // Zero ranges are not valid
        char*  valid  = new char[scanSize];
        float* ranges = new float[scanSize];
	int scan_ind = 0;
	// The lidar has its scan shifted a weird amount from where you would expect.
	// At 0 degree initial pose: int shift = 397; // Found via experimentation
	int shift = 277; // Found via experimentation
	for (const auto& rawScan : rawScans) {
		for (int i = shift; i < rawScan.points.size(); ++i) {
		    if (rawScan.points[i].range != 0.0f) valid[scan_ind] = 1;
		    else valid[scan_ind] = 0;
		    ranges[scan_ind] = rawScan.points[i].range;
		    ++scan_ind;
		}
		for (int i = 0; i < shift; ++i) {
		    if (rawScan.points[i].range != 0.0f) valid[scan_ind] = 1;
		    else valid[scan_ind] = 0;
		    ranges[scan_ind] = rawScan.points[i].range;
		    ++scan_ind;
		}
	}

        resultScan.loadFromVectors(scanSize, ranges, valid);
        resultScan.aperture = M_PI*2*NUM_SCANS; // This is a 360 deg lidar.
	resultScan.rightToLeft = false;

        return resultScan;
    }
}

bool Lidar_MRPT::initialize() {
    ydlidar::os_init();
    if (!setup_lidar_settings(laser)) {
        printf("Lidar settings setup failed, Exiting\n");
        return false;
    }
        
    bool ret = laser.initialize();
        
    if (ret) {
        ret = laser.turnOn();
    } 

    if (!ret){
        fprintf(stderr, "%s\n", laser.DescribeError());
        fflush(stderr);
        return false;
    }
    return true;
}

mrpt::obs::CObservation2DRangeScan Lidar_MRPT::scan() {
	std::vector<LaserScan> all_scans;
    LaserScan CYscan;
    for (int i = 0; i < NUM_SCANS; ++i) {
	    if (!laser.doProcessSimple(CYscan)) {
		fprintf(stderr, "Failed to get Lidar Data\n");
		fflush(stderr);
		return mrpt::obs::CObservation2DRangeScan(); //TODO switch this to optional or something
	    }
	    all_scans.emplace_back(CYscan);
    }


    return getMRPTRangeScanFromScanData(all_scans);
}


bool Lidar_MRPT::tearDown() {
    if (!laser.turnOff()) return false;
    laser.disconnecting();
    return true;
}
