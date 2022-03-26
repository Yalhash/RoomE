#include "RoomeMap.h"
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3D.h>


RoomeMap::RoomeMap() {
    // TODO: update in insert_observation based on how much we've moved
    ICP.options.maxIterations = 100;
    ICP.options.thresholdAng = mrpt::utils::DEG2RAD(10.0f);
    ICP.options.thresholdDist = 0.75;
    ICP.options.ALFA = 0.5f;
    ICP.options.smallestThresholdDist = 0.05f;
    ICP.options.doRANSAC = false;
    ICP.options.ICP_algorithm = mrpt::slam::icpClassic;


}

void RoomeMap::insert_observation(const mrpt::obs::CObservation2DRangeScan& scan,
                                  const mrpt::poses::CPose2D& pose_delta_approx) {

    mrpt::maps::CSimplePointsMap curr_map;
    curr_map.insertObservation(&scan);
    //NOTE: may want run info in future to check goodness etc.
    mrpt::poses::CPosePDF::Ptr pdf = ICP.Align(
            &running_map, 
            &curr_map,
            pose_delta_approx, // pose of m2 relative to m1
            nullptr, // runtime
            nullptr); // run info 
    mrpt::poses::CPosePDFGaussian g_pdf;
    g_pdf.copyFrom(*pdf);
    
    // update current pose to where we think we are
    current_pose += g_pdf.mean; 
    std::cout << "cpose: " << current_pose << " g_pdf: "  << g_pdf.mean << " pose_delta_approx " << pose_delta_approx<< std::endl;
    // correct our scan
    curr_map.changeCoordinatesReference(current_pose);
    // update the current map and grid
    running_map.fuseWith(&curr_map);
    mrpt::poses::CPose3D current_3D(current_pose);
    running_grid.insertObservation(&scan,&current_3D);
}


void RoomeMap::save_to_text_file(const std::string& filename) { 
    running_map.save2D_to_text_file(filename);
    running_grid.saveAsBitmapFile(filename+".bmp");
}


mrpt::poses::CPose2D RoomeMap::get_pose() {
    return current_pose; 
}

mrpt::maps::COccupancyGridMap2D RoomeMap::get_grid_map() {
    return running_grid;
}
