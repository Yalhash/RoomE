#include "RoomeMap.h"
#include "TestEnv.h"

int main() {
    RoomeMap r_map;
    TestEnv env("assets/SimpleEnv.bmp");

    // Insert the base observation
    auto scan1 = env.scan();
    r_map.insert_observation(scan1, mrpt::poses::CPose2D(0,0,0));
    int count = 0;
    while (true) {
        std::string user_input, y, phi;
        std::cout << "enter \"x y phi\" change relative to the last pose" << std::endl;
        std::cout << "or enter \"q\" to quit" << std::endl;
        std::cin >> user_input;
        if (user_input == "q") {
            break;
        } else {
            std::cin >> y >> phi;
            mrpt::poses::CPose2D pose_delta(std::stof(user_input),std::stof(y), std::stof(phi));

            // Take scan 
            auto currScan = env.scan();
            // insert the scan and update the map
            r_map.insert_observation(currScan, pose_delta);  
            env.update_pose(r_map.get_pose());
            // save scan
            std::cout << r_map.get_pose() << std::endl;
            std::string output_name = "scan_" + std::to_string(count);
            std::cout << "-> Saving current map as build/" << output_name << std::endl;
            r_map.save_to_text_file(output_name);
        }
        ++count;
    }
    return 0;
}
