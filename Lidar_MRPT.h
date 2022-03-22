/*
 * A class which encapsulates the lidar scanner
 */

#ifndef LIDAR_MRPT_H
#define LIDAR_MRPT_H

#include "CYdLidar.h"
#include <mrpt/obs/CObservation2DRangeScan.h>

class Lidar_MRPT {
public:

    /*
     * Set up the settings, initialize and start the lidar
     * Returns true on success, false otherwise
     */
    bool initialize();

    /*
     * Takes a scan and converts it into an MRPT scan
     */
    mrpt::obs::CObservation2DRangeScan scan();

    /* 
     * Turn off and disconnect the laser
     * Returns true on success, false otherwise
     */
    bool tearDown();


private:
    CYdLidar laser;
};

#endif
