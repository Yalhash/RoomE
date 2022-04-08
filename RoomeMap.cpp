#include "RoomeMap.h"
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/gui/CDisplayWindow3D.h>

RoomeMap::RoomeMap() {
    current_pose = mrpt::poses::CPose2D(0.0f,0.0f,0);
    //running_map.changeCoordinatesReference(current_pose);
    // TODO: update in insert_observation based on how much we've moved
    ICP.options.maxIterations = 100;
    ICP.options.thresholdAng = mrpt::utils::DEG2RAD(10.0f);
    ICP.options.thresholdDist = 0.75f;
    ICP.options.ALFA = 0.5f;
    ICP.options.smallestThresholdDist = 0.02f;
    ICP.options.doRANSAC = true;
    ICP.options.ICP_algorithm =mrpt::slam::icpLevenbergMarquardt;// mrpt::slam::icpClassic;
}

void RoomeMap::insert_observation(const mrpt::obs::CObservation2DRangeScan& scan,
                                  const mrpt::poses::CPose2D& pose_delta_approx) {

    //update current pose to where we approximately are based on odometry values
    current_pose = mrpt::poses::CPose2D(current_pose.m_coords[0] + pose_delta_approx.m_coords[0], 
                                        current_pose.m_coords[1] + pose_delta_approx.m_coords[1],
                                        current_pose.phi() + pose_delta_approx.phi());
    current_pose.normalizePhi();

    //Create map to represent current scan, and change its frame of reference to the updated current_pose (approximately where it was taken from)
    mrpt::maps::CSimplePointsMap curr_map;
    curr_map.insertObservation(&scan);
    curr_map.changeCoordinatesReference(current_pose);
    

    //NOTE: may want run info in future to check goodness etc.
    //Calculate the error for joining the two maps
    mrpt::poses::CPosePDF::Ptr pdf = ICP.Align(
            &curr_map,
            &running_map, 
            mrpt::poses::CPose2D(0,0,0), 
            nullptr, // runtime
            &info); // run info 

    
    auto mean_pose = pdf->getMeanVal(); 

    //change the incoming map based on icp result
    curr_map.changeCoordinatesReference(mean_pose);
    running_map.fuseWith(&curr_map);

    //push info into the running grid
    mrpt::poses::CPose3D current_3D(current_pose);
    running_grid.insertObservation(&scan,&current_3D);

    

}


void RoomeMap::save_points_to_file(const std::string& filename) { 
    running_map.save2D_to_text_file(filename+".txt");
}

void RoomeMap::save_grid_to_file(const std::string& filename) { 


    for (auto& p : saved_points) {
        int x = running_grid.x2idx(p.first);
        int y = running_grid.y2idx(p.second);
        for (int i = -10; i <= 10; ++i) {
            for (int j = -10; j <= 10; ++j) {
                running_grid.updateCell(x + i, y + j,0);
            }
        }
    }
    running_grid.saveAsBitmapFile(filename + ".bmp");
}


mrpt::poses::CPose2D RoomeMap::get_pose() const {
    return current_pose; 
}

mrpt::maps::COccupancyGridMap2D RoomeMap::get_grid_map() {
    return running_grid;
}

void RoomeMap::save_point(double x, double y) {
    saved_points.emplace_back(std::make_pair(x, y));
}
