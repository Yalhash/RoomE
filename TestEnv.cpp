#include "TestEnv.h"

TestEnv::TestEnv(const std::string& filename) {
    virtual_grid.loadFromBitmapFile(filename, 0.5f);
}


mrpt::obs::CObservation2DRangeScan scan() {
    mrpt::obs::CObservation2DRangeScan result;
    return virtual_grid.laserScanSimulator(&result, 
}

void TestEnv:: update_pose(const mrpt::poses::CPose2D& new_pose) {
    currentPosition = new_pose;
}


