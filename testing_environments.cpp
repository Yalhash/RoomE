#include "RoomeMap.h"
#include "RoomeNav.h"
#include "TestEnv.h"

int main() {
    RoomeMap r_map;
    RoomeNav nav;
    TestEnv env("assets/shapes.png");
    mrpt::poses::CPose2D virtual_pose(0,0,0);
    if (!env.is_map_loaded()) {
        std::cout << "map did not load correctly" 
                  << std::endl;
        return -1;
    }

    // Insert the base observation
    auto scan1 = env.scan();
    r_map.insert_observation(scan1, mrpt::poses::CPose2D(0,0,0));
    int count = 0;
    std::string output_name =  std::to_string(count++) + "_scan";
    std::cout << "-> Saving current map as build/" << output_name << std::endl;
    r_map.save_grid_to_file(output_name);
    while (true) {
        r_map.save_point(r_map.get_pose().m_coords[0], r_map.get_pose().m_coords[1]);
        auto opt_next_point = nav.find_destiny(r_map.get_grid_map(), r_map.get_pose());

        if (!opt_next_point) {
            std::cout << "Couldn't find a next point on " << count << "th" << std::endl;
            break;
        }
        auto destination_point = *opt_next_point;
        mrpt::poses::CPose2D dest_pose(destination_point.x, 
                                       destination_point.y,
                                       r_map.get_pose().phi());

        auto opt_path = nav.find_path(r_map.get_grid_map(), r_map.get_pose(), dest_pose);
        if (!opt_path) {
            std::cout << "Couldn't find a path to that point " << count << "th" << std::endl;
            // break;
        }
        // auto path = *opt_path;

        /*
        std::cout << "Going through path" << std::endl;
        for (const auto& next_point : path) {

            std::cout << "next: " << next_point << std::endl;

            mrpt::poses::CPose2D absolute_new_pose(next_point.x,next_point.y,0);
            auto pose_delta = absolute_new_pose - virtual_pose;
            std::cout << "three things: " << absolute_new_pose << ", " 
                      << r_map.get_pose() << ", " << pose_delta << std::endl;

            // Update location exactly 
            env.update_pose(absolute_new_pose);
            // Take scan
            auto currScan = env.scan();
            // insert the scan and update the map
            r_map.insert_observation(currScan, pose_delta);  
            virtual_pose = absolute_new_pose;
            // save scan
            std::cout << r_map.get_pose() << std::endl;

            output_name = std::to_string(count++) + "_scan" ;
            std::cout << "-> Saving current map as build/" << output_name << std::endl;
            r_map.save_grid_to_file(output_name);

        }
        */
        auto next_point = destination_point;
        std::cout << "next: " << next_point << std::endl;

        mrpt::poses::CPose2D absolute_new_pose(next_point.x,next_point.y,0);
        auto pose_delta = absolute_new_pose - virtual_pose;

        // Update location exactly 
        env.update_pose(absolute_new_pose);
        // Take scan
        auto currScan = env.scan();
        // insert the scan and update the map
        r_map.insert_observation(currScan, pose_delta);  
        virtual_pose = absolute_new_pose;
        // save scan
        std::cout << r_map.get_pose() << std::endl;

        output_name = std::to_string(count++) + "_scan" ;
        std::cout << "-> Saving current map as build/" << output_name << std::endl;
        r_map.save_grid_to_file(output_name);
        if (count > 20) {
            break;
        }
        
    }
    return 0;
}
