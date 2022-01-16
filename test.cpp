/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    *  POSSIBILITY OF SUCH DAMAGE.
    *********************************************************************/
    // standard libs
#include <iostream>
#include <string>
#include <algorithm>
#include <array>
#include <cctype>
    // MRPT includes
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/math/utils.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
    // Lidar includes
#include "CYdLidar.h"
    // local includes
#include "scanData.h"


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
      float frequency = 8.0;

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
      optval = isSingleChannel ? 3 : 4;
      laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
      /// abnormal count
      optval = 4;
      laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

      //////////////////////bool property/////////////////
      /// fixed angle resolution
      bool b_optvalue = false;
      laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
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
      f_optvalue = 64.f;
      laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
      f_optvalue = 0.05f;
      laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
      /// unit: Hz
      laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

      return true;

    }

    mrpt::obs::CObservation2DRangeScan getMRPTRangeScanFromScanData(const LaserScan& rawScan) {
        mrpt::obs::CObservation2DRangeScan resultScan;
        size_t scanSize = rawScan.points.size();
        // Assuming ranges all are valid
        char*  valid  = new char[scanSize];
        for (size_t i = 0; i < scanSize; ++i) valid[i] = 1;

        // copy the vector elements into our new array
        // NOTE: We may want to move this instead of copy for speed
        float* ranges = new float[scanSize];
        std::transform(rawScan.points.begin(), rawScan.points.end(), ranges,
                [](const LaserPoint& lp){return lp.range;});


        resultScan.loadFromVectors(scanSize, ranges, valid);

        resultScan.aperture = M_PI*2; // This is a 360 deg lidar.

        return resultScan;
    }

    int main(int argc, char *argv[]) {
        printf("starting\n");
        printf("\n");
        fflush(stdout);
        ydlidar::os_init();
        
        
        CYdLidar laser;
        if (!setup_lidar_settings(laser)) {
            printf("Lidar settings setup failed, Exiting\n");
            return 1;
        }
        
        bool ret = laser.initialize();
        
        if (ret) {
            ret = laser.turnOn();
        } else {
            fprintf(stderr, "%s\n", laser.DescribeError());
            fflush(stderr);
        }
        
        LaserScan scan;



        if (!laser.doProcessSimple(scan)) {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
            return 1;
        }

        auto scan1 = getMRPTRangeScanFromScanData(scan);

        std::string pause_str;
        std::cout << "enter to take a second scan" << std::endl;
        std::cin >> pause_str;

        if (!laser.doProcessSimple(scan)) {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
            return 1;
        }
        auto scan2 = getMRPTRangeScanFromScanData(scan);

        // Now make simplepoints map from the scans
        mrpt::maps::CSimplePointsMap m1, m2;

        m1.insertObservation(&scan1);
        m2.insertObservation(&scan2);



        // set icp alg parameters
        mrpt::slam::CICP ICP;
        mrpt::slam::CICP::TReturnInfo info; // info upon return of algorithm
        ICP.options.ICP_algorithm = mrpt::slam::icpClassic;
        
        // TODO: find out units and perhaps alter these default settings
        ICP.options.maxIterations = 100;
        ICP.options.thresholdAng = mrpt::utils::DEG2RAD(10.0f);
        ICP.options.thresholdDist = 0.75f;
        ICP.options.ALFA = 0.5f;
        ICP.options.smallestThresholdDist = 0.05f;
        ICP.options.doRANSAC = false;

        // ICP.options.dumpToConsole();
        

        // initial pose (position and orientation)
        mrpt::poses::CPose2D initialPose(0.0f, 0.0f, (float)mrpt::utils::DEG2RAD(0.0f));
        float runtime;

        // align the two maps (the main algorithm)
        mrpt::poses::CPosePDF::Ptr pdf = ICP.Align(&m1, &m2, initialPose, &runtime, &info);
        printf(
            "ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n "
            "-> ",
            runtime * 1000, info.nIterations,
            runtime * 1000.0f / info.nIterations,
            info.goodness * 100);
        std::cout << "Mean of estimation: " << pdf->getMeanVal() << std::endl << std::endl;

        mrpt::poses::CPosePDFGaussian gPdf;
        gPdf.copyFrom(*pdf);
        std::cout << "Covariance of estimation: " << std::endl << gPdf.cov << std::endl;
        std::cout << " std(x): " << sqrt(gPdf.cov(0, 0)) << std::endl;
        std::cout << " std(y): " << sqrt(gPdf.cov(1, 1)) << std::endl;
        std::cout << " std(phi): " << mrpt::utils::RAD2DEG(sqrt(gPdf.cov(2, 2))) << " (deg)" << std::endl;


        // Saving result:
        std::cout << "-> Saving reference map as out/scan1.txt" << std::endl;
        m1.save2D_to_text_file("scan1.txt");

        std::cout << "-> Saving map to align as out/scan2.txt" << std::endl;
        m2.save2D_to_text_file("scan2.txt");

        std::cout << "-> Saving transformed map to align as out/scan2_trans.txt" << std::endl;
        auto m2_trans = m2;
        m2_trans.changeCoordinatesReference(gPdf.mean);
        m2_trans.save2D_to_text_file("scan2_trans.txt");

        mrpt::math::CMatrixFloat COV22 = mrpt::math::CMatrixFloat(mrpt::math::CMatrixDouble(gPdf.cov));
        COV22.setSize(2, 2);
        mrpt::math::CVectorFloat MEAN2D(2);
        MEAN2D[0] = gPdf.mean.x();
        MEAN2D[1] = gPdf.mean.y();
        {
            std::ofstream f("view_ellip.m");
            f << mrpt::math::MATLAB_plotCovariance2D(COV22, MEAN2D, 3.0f);
        }


    laser.turnOff();
    laser.disconnecting();
    return 0;
}
