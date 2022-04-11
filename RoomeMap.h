#ifndef ROOME_MAP_H
#define ROOME_MAP_H
#include <mrpt/poses/CPose2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <utility>
#include <vector>

class RoomeMap {
public:
    RoomeMap();

    void initial_observation(const mrpt::obs::CObservation2DRangeScan& init_scan);
    // Insert the observation and correct the pose approximation
    void insert_observation(const mrpt::obs::CObservation2DRangeScan& scan,
                            const mrpt::poses::CPose2D& pose_delta_approx);

    void save_points_to_file(const std::string& filename);
    void save_grid_to_file(const std::string& filename);

    mrpt::poses::CPose2D get_pose() const;

    mrpt::maps::COccupancyGridMap2D get_grid_map();
    mrpt::maps::CSimplePointsMap get_points_map();

private:
    mrpt::slam::CMetricMapBuilderICP icp_map;
    mrpt::obs::CActionRobotMovement2D::TMotionModelOptions opts;
};

#endif
