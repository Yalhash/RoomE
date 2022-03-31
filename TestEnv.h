#ifndef TEST_ENV_H
#define TEST_ENV_H

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <vector> 
#include <utility> 
#include <mrpt/poses/CPose2D.h>

class TestEnv {
public:
    TestEnv(const std::string& filename);
    mrpt::obs::CObservation2DRangeScan scan();
    void update_pose(const mrpt::poses::CPose2D& new_pose);
    bool is_map_loaded();
private:
    mrpt::maps::COccupancyGridMap2D virtual_grid;
    mrpt::poses::CPose2D currentPosition;
};

#endif
