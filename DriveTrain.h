#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H

#include <utility>

// MRPT includes
#include <mrpt/poses/CPose2D.h>

// Local includes
#include "ArduinoSerial.h"

class DriveTrain {
public:
    // Set up the serial
    DriveTrain();
    // Tear down the serial
    ~DriveTrain();

    // Move RoomE from the start pose to the finish point. 
    // return the left and right wheel odometry info.
    // (Note, maybe this should return the approximate new pose.
    std::pair<int, int> move(mrpt::poses::CPose2D start, mrpt::math::TPoint2D finish);

private:
    ArduinoSerial serial;
};

#endif
