#ifndef ROOME_MAP_H
#define ROOME_MAP_H
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/slam/CICP.h>
#include <utility>
#include <vector>

class RoomeMap {
public:
    RoomeMap();

    // Insert the observation and correct the pose approximation
    void insert_observation(const mrpt::obs::CObservation2DRangeScan& scan,
                            const mrpt::poses::CPose2D& pose_delta_approx);

    void save_points_to_file(const std::string& filename);
    void save_grid_to_file(const std::string& filename);

    mrpt::poses::CPose2D get_pose() const;

    mrpt::maps::COccupancyGridMap2D get_grid_map();
    void save_point(double x, double y);

private:
    mrpt::poses::CPose2D current_pose;
    mrpt::slam::CICP ICP;
    mrpt::slam::CICP::TReturnInfo info;
    mrpt::maps::CSimplePointsMap running_map;
    mrpt::maps::COccupancyGridMap2D running_grid;
    std::vector<std::pair<double,double>> saved_points;
};

#endif
