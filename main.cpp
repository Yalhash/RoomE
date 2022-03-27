// standard libs
#include <iostream>
#include <string>
#include <cctype>
// MRPT includes
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/math/utils.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
// Lidar includes
#include "CYdLidar.h"
// local includes
#include "ArduinoSerial.h"
#include "Lidar_MRPT.h"
#include "RoomeMap.h"
#include "DriveTrain.h"


int main() {
    Lidar_MRPT lidar;
    if (!lidar.initialize()) {
        printf("Lidar initialization failed! Exiting.");
    }

    RoomeMap r_map;
    RoomeNav nav;
    DriveTrain d_train;
    // Base scan
    auto base_scan = lidar.scan();
    r_map.insert_observation(scan1, mrpt::poses::CPose2D(0,0,0));
    while (true) {
        // 1. Find point to move to (Or end)
        mrpt::math::TPoint2D move_point; 
        // 2. Find path to that point
        // 3. Select next point in path
        // 4. Move to that point (possibly doing step 5 along the way)
        auto pose_delta = d_train.move(r_map.get_pose(), move_point);
        // Repeat 3/4 until end of path
        // 5. Insert the scan, and correct any errors
        auto currScan = lidar.scan();
        r_map.insert_observation(currScan, pose_delta);  
    }
}
