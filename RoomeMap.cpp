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

    /*    if (running_map.isEmpty()){
        std::cout << "First observation, icp not required" << std::endl;
        running_map.insertObservation(&scan);
        mrpt::poses::CPose3D acurrent3D(current_pose);
        running_grid.insertObservation(&scan,&acurrent3D);
        return;
    }*/
    mrpt::maps::CSimplePointsMap curr_map;
    /* mrpt::poses::CPose3D obs_pose(current_pose + pose_delta_approx); */
/*    curr_map.save2D_to_text_file("icpBS/beforePts.txt");
    curr_map.changeCoordinatesReference(pose_delta_approx + current_pose);
    curr_map.save2D_to_text_file("icpBS/refChange.txt");*/
    curr_map.changeCoordinatesReference(mrpt::poses::CPose2D(0,0,-pose_delta_approx.phi() - current_pose.phi()));

    curr_map.insertObservation(&scan);

    running_map.changeCoordinatesReference(mrpt::poses::CPose2D(0,0,- current_pose.phi()));
    /* curr_map.insertObservation(&scan, &obs_pose); */
    //NOTE: may want run info in future to check goodness etc.
    mrpt::poses::CPosePDF::Ptr pdf = ICP.Align(
            &running_map, 
            &curr_map,
            mrpt::poses::CPose2D(pose_delta_approx.m_coords[0],pose_delta_approx.m_coords[1],0), // pose of m2 relative to m1
            nullptr, // runtime
            &info); // run info 

    std::cout << "ICP iterations: " << info.nIterations << " goodness: " << info.goodness << std::endl;
    
    // update current pose to where we think we are
    auto mean_pose = pdf->getMeanVal(); 

    //If icp algorithm isnt good enough use odometry
    if (info.goodness >= 0){
        /* current_pose.m_coords[0] += mean_pose.m_coords[0]; */
        /* current_pose.m_coords[1] += mean_pose.m_coords[1]; */
        /* current_pose.phi_incr(mean_pose.phi()); */
        std::cout << "cpose: " << current_pose << std::endl;
        std::cout << "mean pose: " << mean_pose << std::endl;
        std::cout << "delta: " << pose_delta_approx << std::endl;
    
    std::cout << "curr pose + delta pose: " << current_pose+pose_delta_approx << std::endl;
        std::cout <<  " current + mean: " << current_pose+mean_pose << std::endl;
        std::cout << "current - mean: " << current_pose - mean_pose << std::endl;
        
        std::cout << "mean -current: " << mean_pose - current_pose << std::endl;
        
        std::cout << " mean + current: " << mean_pose + current_pose << std::endl;
        std::cout << "-current + mean: " << -current_pose + mean_pose << std::endl;
        std::cout << "-current - mean: " << -current_pose - mean_pose << std::endl;
        
        
       /* current_pose.m_coords[0] += pose_delta_approx.m_coords[0];
        current_pose.m_coords[1] += pose_delta_approx.m_coords[1];
        current_pose.phi_incr(pose_delta_approx.phi());*/
        
        std::cout << "--------------------------------------------" << std::endl;
        // correct our scan
        //curr_map.changeCoordinatesReference(mean_pose);
        // update the current map and grid

        running_map.changeCoordinatesReference(-mean_pose);
        running_map.fuseWith(&curr_map);
       // save_points_to_file(std::to_string(count)+"_Fused");
        running_map.changeCoordinatesReference(mrpt::poses::CPose2D(0,0,current_pose.phi()));
        save_points_to_file("Fused_adjusted");
        //count ++;
       current_pose = mean_pose + current_pose;
       /* mrpt::poses::CPose3D current_3D( mrpt::poses::CPose2D(running_grid.x2idx(current_pose.m_coords[0]),running_grid.x2idx(current_pose.m_coords[1]),current_pose.phi())); */
        mrpt::poses::CPose3D current_3D(current_pose);
        running_grid.insertObservation(&scan,&current_3D);
    }else{
        current_pose.m_coords[0] += pose_delta_approx.m_coords[0];
        current_pose.m_coords[1] += pose_delta_approx.m_coords[1];
        current_pose.phi_incr(pose_delta_approx.phi());
        std::cout << "bad icp. Discarding and using odometry values" << std::endl;
    }

    //std::cout << "cpose: " << current_pose << " pdf mean: "  << mean_pose << " pose_delta_approx " << pose_delta_approx<< std::endl;
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
