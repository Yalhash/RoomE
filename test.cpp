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

//navtesting includes
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace std;

int navTesting(){
    filename = "assets/Env_asset2.png";
    printf("starting\n");
    printf("\n");
    fflush(stdout);

    mrpt::maps::COccupancyGridMap2D occ_grid;
	    

    
    if (occ_grid.loadFromBitmapFile(filename, 0.5f)){
    	cout << "Building Voronoi diagram...\n";
        occ_grid.buildVoronoiDiagram(0.5f, 0.3f);

        CImage img_grid;
        occ_grid.getAsImage(img_grid);

        CImage img_voronoi;
        CMatrixDouble mat_voronoi;
        occ_grid.getVoronoiDiagram().getAsMatrix(mat_voronoi);
        img_voronoi.setFromMatrix(mat_voronoi, false /* do normalization */);

        // Show results:
        CDisplayWindow win1("Grid map");
        win1.showImage(img_grid);

        CDisplayWindow win2("Voronoi map");
        win2.showImage(img_voronoi);

        mrpt::system::pause();
    }



}

int demo() {
    printf("starting\n");
    printf("\n");
    fflush(stdout);

    Lidar_MRPT lidar;
    lidar.initialize();
	
    RoomeMap r_map;

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
            r_map.save_points_to_file(user_input);
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
    return navTesting();
}
