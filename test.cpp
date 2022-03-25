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


int demo() {
    printf("starting\n");
    printf("\n");
    fflush(stdout);

    Lidar_MRPT lidar;
    lidar.initialize();
	
    RoomeMap r_map;
    mrpt::maps::COccupancyGridMap2D occ_grid;

    // Insert the base observation
    auto scan1 = lidar.scan();
    r_map.insert_observation(scan1, mrpt::poses::CPose2D(0,0,0));
    occ_grid = r_map.get_grid_map();
    while (true) {
        std::string user_input, y, phi;
        std::cout << "enter \"x y phi\" change relative to the last pose" << std::endl;
        std::cout << "or enter \"s <filename>\" to save the current map to a file" << std::endl;
        std::cout << "or enter \"q\" to quit" << std::endl;
        std::cin >> user_input;
        if (user_input == "q") {
            break;
        } else if (user_input == "s") {
            std::cin >> user_input;
            // save to filename
            std::cout << "-> Saving current map as out/" << user_input << std::endl;
            r_map.save_to_text_file(user_input);
        } else {
            std::cin >> y >> phi;
            mrpt::poses::CPose2D pose_delta(std::stof(user_input),std::stof(y), std::stof(phi));

            // Take scan 
            auto currScan = lidar.scan();
            // insert the scan and update the map
            r_map.insert_observation(currScan, pose_delta);  
        }
    }
    lidar.tearDown();
    return 0;
}

int main(int argc, char *argv[]) {
    return demo();
}
