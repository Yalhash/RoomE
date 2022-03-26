#ifndef ROOME_MAP_H
#define ROOME_MAP_H
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/slam/CICP.h>

class RoomeMap {
public:
    RoomeMap();

    // Insert the observation and correct the pose approximation
    void insert_observation(const mrpt::obs::CObservation2DRangeScan& scan,
                            const mrpt::poses::CPose2D& pose_delta_approx);

    void save_points_to_file(const std::string& filename);
    void save_grid_to_file(const std::string& filename);

    mrpt::poses::CPose2D get_pose();

    mrpt::maps::COccupancyGridMap2D get_grid_map();

private:
    mrpt::poses::CPose2D current_pose;
    mrpt::slam::CICP ICP;
    mrpt::maps::CSimplePointsMap running_map;
    mrpt::maps::COccupancyGridMap2D running_grid;
};

#endif
