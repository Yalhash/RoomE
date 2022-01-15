#ifndef SCANDATA_H
#define SCANDATA_H

#include "CYdLidar.h"
#include <vector>

struct ScanData {
    std::vector<LaserPoint> points;
    float x,y;
};

#endif
