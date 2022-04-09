#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H

#include <utility>

// MRPT includes
#include <mrpt/poses/CPose2D.h>

// Local includes
#include "ArduinoSerial.h"
class RoomeTest;

class DriveTrain {
public:
    // Set up the serial
    DriveTrain();
    // Tear down the serial
    ~DriveTrain();

    //calculates the required turn and drive vectors and then turns. 
    //Returns the approximate change in x,y,phi based on odometry info
    //Call post_scan_drive() afterwards to move the calculated amount
    mrpt::poses::CPose2D calculate_and_turn(mrpt::poses::CPose2D start, mrpt::math::TPoint2D finish);

    // Moves the roome straight by the previously calculated amount. 
    // Returns the approximate change in x,y,phi based on odometry info
    // can only be called after calculate_and_turn has been run
    mrpt::poses::CPose2D post_scan_drive();

private:
    ArduinoSerial serial;
    friend RoomeTest;
    std::string calculated_straight_movement;
    bool movement_calculated;
};

#endif
