// standard libs
#include <iostream>
#include <string>
#include <algorithm>
#include <array>
#include <unistd.h>
#include <cctype>
// MRPT includes
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/math/utils.h>
#include <mrpt/maps/CSimplePointsMap.h>
//#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
// Lidar includes
#include "CYdLidar.h"
// local includes
#include "ArduinoSerial.h"
#include "Lidar_MRPT.h"


int demo() {
    printf("starting\n");
    printf("\n");
    fflush(stdout);

    Lidar_MRPT lidar;
    lidar.initialize();
	
    auto scan1 = lidar.scan();
    mrpt::maps::CSimplePointsMap baseMap, currMap;
    baseMap.insertObservation(&scan1);

    // set icp alg parameters
    mrpt::slam::CICP ICP;
    mrpt::slam::CICP::TReturnInfo info; // info upon return of algorithm
    ICP.options.ICP_algorithm = mrpt::slam::icpClassic;
    
    // TODO: find out units and perhaps alter these default settings
    ICP.options.maxIterations = 100;
    ICP.options.thresholdAng = mrpt::utils::DEG2RAD(10.0f);
    ICP.options.thresholdDist = 0.5f;
    ICP.options.ALFA = 0.5f;
    ICP.options.smallestThresholdDist = 0.03f;
    ICP.options.doRANSAC = false;
    float runtime;

	mrpt::poses::CPose2D currentPose;
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
            baseMap.save2D_to_text_file(user_input);
        } else {
            std::cin >> y >> phi;
            mrpt::poses::CPose2D poseDelta(std::stof(user_input),std::stof(y), std::stof(phi));
            currentPose += poseDelta;
            auto cPose3D = mrpt::poses::CPose3D(currentPose);
            auto currScan = lidar.scan(&cPose3D);

            currMap.insertObservation(&currScan);
            mrpt::poses::CPosePDF::Ptr pdf = ICP.Align(&baseMap, &currMap, currentPose, &runtime, &info);

            printf(
                "ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n "
                "-> ",
                runtime * 1000, info.nIterations,
                runtime * 1000.0f / info.nIterations,
                info.goodness * 100);
            std::cout << "Mean of estimation: " << pdf->getMeanVal() << std::endl << std::endl;
            mrpt::poses::CPosePDFGaussian gPdf;
            gPdf.copyFrom(*pdf);
                        currMap.changeCoordinatesReference(gPdf.mean);
            baseMap.fuseWith(&currMap);
            currMap.clear();
        }
    }
    lidar.tearDown();
    return 0;
}

int main(int argc, char *argv[]) {
    return demo();
}
