#include "DriveTrain.h"

DriveTrain::DriveTrain() {
    serial.usb_open();
}

DriveTrain::~DriveTrain() {
    serial.usb_close();
}

std::pair<int, int> DriveTrain::move(mrpt::poses::CPose2D start, mrpt::math::TPoint2D finish) {
    int left_wheel, right_wheel;

    // TODO move RoomE and fill left wheel and right wheel with the l/r odometry info

    return std::make_pair(left_wheel, right_wheel);
}
