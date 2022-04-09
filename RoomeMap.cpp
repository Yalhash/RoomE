#include "RoomeMap.h"
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/obs/CSensoryFrame.h>

RoomeMap::RoomeMap() {
    // ICP options
    icp_map.ICP_params.maxIterations = 100;
    icp_map.ICP_params.thresholdAng = mrpt::utils::DEG2RAD(10.0f);
    icp_map.ICP_params.thresholdDist = 0.75f;
    icp_map.ICP_params.ALFA = 0.5f;
    icp_map.ICP_params.smallestThresholdDist = 0.02f;
    icp_map.ICP_params.doRANSAC = true;
    icp_map.ICP_params.ICP_algorithm =mrpt::slam::icpClassic;
    mrpt::maps::TMetricMapInitializerPtr grid_def(mrpt::maps::COccupancyGridMap2D::MapDefinition());
    mrpt::maps::TMetricMapInitializerPtr points_def(mrpt::maps::CSimplePointsMap::MapDefinition());
    icp_map.ICP_options.mapInitializers.push_back(grid_def);
    icp_map.ICP_options.mapInitializers.push_back(points_def);

    // Movement model options (Treat our error as a guassian)
    opts.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
}

void RoomeMap::initial_observation(const mrpt::obs::CObservation2DRangeScan& init_scan) {
    mrpt::poses::CPose2D init_pose(0, 0, M_PI/2);
    mrpt::maps::CSimpleMap c_map;
    mrpt::poses::CPosePDFGaussian pose_g2d(init_pose);
    mrpt::poses::CPose3DPDF* pose_g3d = mrpt::poses::CPose3DPDF::createFrom2D(pose_g2d);
    mrpt::obs::CSensoryFrame s_frame;
    
    s_frame.push_back(mrpt::obs::CObservationPtr(init_scan.duplicateGetSmartPtr()));

    c_map.insert(pose_g3d, s_frame);
    icp_map.initialize(c_map, &pose_g2d);
}

void RoomeMap::insert_observation(const mrpt::obs::CObservation2DRangeScan& scan,
                                  const mrpt::poses::CPose2D& pose_delta_approx) {
    mrpt::maps::CSimpleMap c_map;
    mrpt::obs::CActionRobotMovement2D act_mov;
    act_mov.computeFromOdometry(pose_delta_approx, opts);
    act_mov.timestamp = mrpt::system::getCurrentTime();

    mrpt::obs::CSensoryFrame observation;
    
    observation.push_back(mrpt::obs::CObservationPtr(scan.duplicateGetSmartPtr()));

    mrpt::obs::CActionCollection action(act_mov);
    icp_map.processActionObservation(action, observation);
}


void RoomeMap::save_points_to_file(const std::string& filename) { 
    get_points_map().save2D_to_text_file(filename+".txt");
}

void RoomeMap::save_grid_to_file(const std::string& filename) { 
    get_grid_map().saveAsBitmapFile(filename + ".bmp");
}


mrpt::poses::CPose2D RoomeMap::get_pose() const {
    mrpt::poses::CPose3DPDFPtr pose_pdf_3d = icp_map.getCurrentPoseEstimation(); 
    auto pose_3d =  pose_pdf_3d->getMeanVal();
    return mrpt::poses::CPose2D(pose_3d.m_coords[0], pose_3d.m_coords[1], pose_3d.yaw());
}

mrpt::maps::COccupancyGridMap2D RoomeMap::get_grid_map() {
    mrpt::maps::COccupancyGridMap2D grid;
    mrpt::maps::CSimpleMap s_map;

    icp_map.getCurrentlyBuiltMap(s_map);
    grid.loadFromProbabilisticPosesAndObservations(s_map);
    return grid;
}

mrpt::maps::CSimplePointsMap RoomeMap::get_points_map() {
    mrpt::maps::CSimplePointsMap points;
    mrpt::maps::CSimpleMap s_map;

    icp_map.getCurrentlyBuiltMap(s_map);
    points.loadFromProbabilisticPosesAndObservations(s_map);
    return points;

}
