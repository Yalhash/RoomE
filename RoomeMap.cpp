#include "RoomeMap.h"
#include <mrpt/poses/CPosePDF.h>


RoomeMap::RoomeMap() {
    // TODO: find out units and perhaps alter these default settings
    ICP.options.maxIterations = 100;
    ICP.options.thresholdAng = mrpt::utils::DEG2RAD(5.0f);
    ICP.options.thresholdDist = 0.5f;
    ICP.options.ALFA = 0.5f;
    ICP.options.smallestThresholdDist = 0.03f;
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
    
    // correct our scan
    curr_map.changeCoordinatesReference(g_pdf.mean);
    // update current pose to where we think we are
    current_pose += g_pdf.mean; 
    // update the current map 
    running_map.fuseWith(&curr_map);
}
