#include "TestEnv.h"
#include <iostream>

TestEnv::TestEnv(const std::string& filename) {
   
    bool result =  virtual_grid.loadFromBitmapFile(filename, 0.5f);
}


mrpt::obs::CObservation2DRangeScan TestEnv::scan() {
    mrpt::obs::CObservation2DRangeScan result;
    result.aperture = M_PI*2;
    virtual_grid.laserScanSimulator(result, currentPosition);
    // simulate 360 scan
    return result;
}

void TestEnv::update_pose(const mrpt::poses::CPose2D& new_pose) {
    // std::cout << "updating: " << 
    currentPosition = new_pose;
}


