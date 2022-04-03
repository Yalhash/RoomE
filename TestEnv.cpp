#include "TestEnv.h"
#include <iostream>

TestEnv::TestEnv(const std::string& filename) {
   
    bool result =  virtual_grid.loadFromBitmapFile(filename, 0.05f);
    virtual_grid.insertionOptions.wideningBeamsWithDistance = true;
}

bool TestEnv::is_map_loaded() {
    return !virtual_grid.isEmpty();
}


mrpt::obs::CObservation2DRangeScan TestEnv::scan() {
    mrpt::obs::CObservation2DRangeScan result;
    result.aperture = M_PI*2;
    virtual_grid.laserScanSimulator(result, currentPosition, 0.5, 1440);
    /* virtual_grid.laserScanSimulator(result, currentPosition); */
    // simulate 360 scan
    return result;
}

void TestEnv::update_pose(const mrpt::poses::CPose2D& new_pose) {
    currentPosition = new_pose;
}



