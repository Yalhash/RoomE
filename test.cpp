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

int driveTrainTest(){
    printf("starting\n");
    printf("\n");
    fflush(stdout);
	
    DriveTrain d_train;

    d_train.turn_right();
    d_train.go_straight();
    d_train.turn_left();
    


}
int demo() {
    printf("starting\n");
    printf("\n");
    fflush(stdout);

    Lidar_MRPT lidar;
    lidar.initialize();
	
    RoomeMap r_map;
    DriveTrain d_train;

    // Insert the base observation
    auto scan1 = lidar.scan();
    r_map.insert_observation(scan1, mrpt::poses::CPose2D(0,0,0));
    int count = 0;
    std::string output_name = "scan_" + std::to_string(count);
    std::cout << "-> Saving initial map as build/" << output_name << std::endl;
    r_map.save_grid_to_file(output_name);
    r_map.save_points_to_file(output_name + ".txt");
    while (true) {
        std::string user_input, y, phi;
        std::cout << "enter \"x y\" change relative to the last pose" << std::endl;
        std::cout << "or enter \"s <filename>\" to save the current map to a file" << std::endl;
        std::cout << "or enter \"q\" to quit" << std::endl;
        std::cin >> user_input;
        if (user_input == "q") {
            break;
        } else {
            std::cin >> y;
            auto curr_pose = r_map.get_pose();
            mrpt::math::TPoint2D move_point(
                    curr_pose.m_coords[0] + std::stof(user_input),
                    curr_pose.m_coords[0] + std::stof(y));
            auto pose_delta = d_train.move(r_map.get_pose(), move_point);


            // Take scan 
            auto currScan = lidar.scan();
            // insert the scan and update the map
            r_map.insert_observation(currScan, pose_delta);  

            ++count;
            std::string output_name = "scan_" + std::to_string(count);
            std::cout << "-> Saving current map as build/" << output_name << std::endl;
            r_map.save_grid_to_file(output_name);
            r_map.save_points_to_file(output_name + ".txt");
        }
    }
    lidar.tearDown();
    return 0;
}

int main(int argc, char *argv[]) {
    return driveTrainTest();
}
