#include "RoomeMap.h"
#include "RoomeNav.h"
#include "TestEnv.h"
#include <mrpt/utils/CImage.h>


void save_frontier_map(const std::string& output_path, const std::vector<RoomeNav::Frontier>& fronts, const mrpt::maps::COccupancyGridMap2D& grid, std::pair<double ,double> to_move, std::pair<double, double> roomePos) {
    mrpt::utils::CImage img;
    grid.getAsImage(img, false, true);

    std::cout << "Number of frontiers: " << fronts.size() << std::endl;
    std::cout << "-> Saving frontier map at " << output_path << std::endl;

    for (const auto& front : fronts) {
        for (const auto& p : front.pts) {
                int x = p.first;
                int y = grid.getSizeY() - 1 - p.second;
            img.rectangle(x, y, x, y, mrpt::utils::TColor(255,0,0));
        }
        img.cross(grid.x2idx(front.centroid.x), grid.getSizeY() - 1 - grid.y2idx(front.centroid.y), mrpt::utils::TColor(0,0,255), '+', 10, 3);
    }

    img.cross(grid.x2idx(to_move.first), grid.getSizeY() - 1 -  grid.y2idx(to_move.second), mrpt::utils::TColor(0, 255, 0), 'x', 10, 3);
    int r_pos_x = grid.x2idx(roomePos.first);
    int r_pos_y = grid.getSizeY() - 1 -  grid.y2idx(roomePos.second);
    img.cross(r_pos_x, r_pos_y, mrpt::utils::TColor(255, 0, 255), '+', 20, 3);
    img.saveToFile(output_path);
}

int main() {
    std::string output_name;
    RoomeMap r_map;
    RoomeNav nav;
    TestEnv env("assets/shapes.png");
    mrpt::poses::CPose2D virtual_pose(0,0,M_PI/2);
    env.update_pose(virtual_pose);
    if (!env.is_map_loaded()) {
        std::cout << "map did not load correctly" 
                  << std::endl;
        return -1;
    }

    // Insert the base observation
    auto scan1 = env.scan();
    // 0 Difference scan;
    r_map.insert_observation(scan1, mrpt::poses::CPose2D(0,0,0));
    int count = 0;

    while (true) {
        auto frontiers = nav.find_frontiers(r_map.get_grid_map(), r_map.get_pose());
        // save scan
        output_name = "outputs/" + std::to_string(count++) + "_scan.jpg" ;

        if (frontiers.empty()) {
            std::cout << "Couldn't find a next frontier" << count << "th" << std::endl;
            break;
        }

        // Find the closest point frontier
        int min_ind = 0;
        double min_dist;
        {
            auto centroid = frontiers[0].centroid;
            min_dist = r_map.get_pose().sqrDistanceTo(mrpt::poses::CPose2D(centroid.x, centroid.y, r_map.get_pose().phi()));
            /* min_dist = sqrt( */
            /*                (r_map.get_pose().m_coords[0] - centroid.x) */
            /*               *(r_map.get_pose().m_coords[0] - centroid.x) */ 
            /*               +(r_map.get_pose().m_coords[1] - centroid.y) */
            /*               *(r_map.get_pose().m_coords[1] - centroid.y)); */

        }

        for (int i = 1; i < frontiers.size(); ++i) {
            auto centroid = frontiers[i].centroid;
            auto curr_dist = r_map.get_pose().sqrDistanceTo(mrpt::poses::CPose2D(centroid.x, centroid.y, r_map.get_pose().phi()));
            /* double curr_dist = sqrt( */
            /*                     (r_map.get_pose().m_coords[0] - centroid.x) */
            /*                    *(r_map.get_pose().m_coords[0] - centroid.x) */ 
            /*                    +(r_map.get_pose().m_coords[1] - centroid.y) */
            /*                    *(r_map.get_pose().m_coords[1] - centroid.y)); */
            if (min_dist > curr_dist){
                min_ind = i;
                min_dist = curr_dist;
            }
        }

        auto frontier = frontiers[min_ind];

        auto next_point = frontier.centroid;

        save_frontier_map(output_name, frontiers, r_map.get_grid_map(), std::make_pair(next_point.x, next_point.y), std::make_pair(virtual_pose.m_coords[0], virtual_pose.m_coords[1]));

        mrpt::poses::CPose2D new_pose(next_point.x,next_point.y, virtual_pose.phi());


        mrpt::poses::CPose2D pose_delta = new_pose - virtual_pose;
        /* mrpt::poses::CPose2D pose_delta((new_pose.m_coords[0] - virtual_pose.m_coords[0]), */ 
        /*                                 (new_pose.m_coords[1] - virtual_pose.m_coords[1]), */ 
        /*                                  new_pose.phi() - virtual_pose.phi()); */

        std::cout << "Want to move " << pose_delta << std::endl;
        virtual_pose = new_pose;
        std::cout << "virtual: " << virtual_pose << std::endl;
        // Update location exactly 
        env.update_pose(virtual_pose);
        // Take scan
        auto scan = env.scan();
        // insert the scan and update the map
        r_map.insert_observation(scan, pose_delta);  
        
    }
    save_frontier_map("outputs/final_scan.jpg", {}, r_map.get_grid_map(), std::make_pair(0, 0), std::make_pair(virtual_pose.m_coords[0], virtual_pose.m_coords[1]));
    return 0;
}
