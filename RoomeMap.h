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

    mrpt::poses::CPose2D get_pose() { return current_pose; }

    void save_to_text_file(const std::string& filename) 
    { running_map.save2D_to_text_file(filename); }

private:
    mrpt::poses::CPose2D current_pose;
    mrpt::slam::CICP ICP;
    mrpt::maps::CSimplePointsMap running_map;
};

#endif
