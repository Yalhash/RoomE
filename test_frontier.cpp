#include "RoomeMap.h"
#include "RoomeNav.h"
#include "TestEnv.h"
#include <mrpt/utils/CImage.h>


void add_frontier_info(mrpt::utils::CImage& img, const std::vector<RoomeNav::Frontier>& fronts,
                       const mrpt::maps::COccupancyGridMap2D& grid) {

    std::cout << "Number of frontiers: " << fronts.size() << std::endl;
    for (const auto& front : fronts) {
        for (const auto& p : front.pts) {
                int x = p.first;
                int y = grid.getSizeY() - 1 - p.second;
            img.rectangle(x, y, x, y, mrpt::utils::TColor(255,0,0));
        }
        img.cross(grid.x2idx(front.centroid.x), grid.getSizeY() - 1 - grid.y2idx(front.centroid.y), mrpt::utils::TColor(0,0,255), '+', 10, 3);
    }

}

void add_roome_position(mrpt::utils::CImage& img, const mrpt::poses::CPose2D& roome_pose, const mrpt::maps::COccupancyGridMap2D& grid) {
    int r_pos_x = grid.x2idx(roome_pose.m_coords[0]);
    int r_pos_y = grid.getSizeY() - 1 -  grid.y2idx(roome_pose.m_coords[1]);
    img.cross(r_pos_x, r_pos_y, mrpt::utils::TColor(255, 0, 255), '+', 20, 3);
}

void add_path_nodes(mrpt::utils::CImage& img, const std::deque<mrpt::math::TPoint2D>& travel_path, const mrpt::maps::COccupancyGridMap2D& grid) {
    for (const auto& pt : travel_path) {
        img.cross(grid.x2idx(pt.x), grid.getSizeY() - 1 -  grid.y2idx(pt.y), mrpt::utils::TColor(0, 255, 0), 'x', 10, 3);
    }
}

double get_new_ang(double curr_ang) {
    return std::fmod(curr_ang + M_PI/20.0 , 2*M_PI);
}

RoomeNav::Frontier find_nearest_frontier(const std::vector<RoomeNav::Frontier>& frontiers, const RoomeMap& r_map) {
        // Find the closest point frontier
        int min_ind = 0;
        double min_dist;
        {
            auto centroid = frontiers[0].centroid;
            min_dist = r_map.get_pose().sqrDistanceTo(mrpt::poses::CPose2D(centroid.x, centroid.y, r_map.get_pose().phi()));
        }

        for (int i = 1; i < frontiers.size(); ++i) {
            auto centroid = frontiers[i].centroid;
            auto curr_dist = r_map.get_pose().sqrDistanceTo(mrpt::poses::CPose2D(centroid.x, centroid.y, r_map.get_pose().phi()));

            if (min_dist > curr_dist){
                min_ind = i;
                min_dist = curr_dist;
            }
        }

        return frontiers[min_ind];
}

int main() {
    std::string output_name;
    RoomeMap r_map;
    RoomeNav nav;
    TestEnv env("assets/shapes.png");
    mrpt::poses::CPose2D virtual_pose(0,0,0);
    env.update_pose(virtual_pose);
    if (!env.is_map_loaded()) {
        std::cout << "map did not load correctly" 
                  << std::endl;
        return -1;
    }

    // Insert the base observation
    auto scan1 = env.scan();
    // 0 Difference scan;
    r_map.initial_observation(scan1);
    int count = 0;

    while (true) {
        auto frontiers = nav.find_frontiers(r_map.get_grid_map(), r_map.get_pose());

        if (frontiers.empty()) {
            std::cout << "Couldn't find a next frontier" << count << "th" << std::endl;
            break;
        }

        // Pick frontiers giving priority to closest
	    std::sort(frontiers.begin(), frontiers.end(), 
            [&] (const RoomeNav::Frontier& f1, const RoomeNav::Frontier& f2) {
                return r_map.get_pose().sqrDistanceTo(
                    mrpt::poses::CPose2D(f1.centroid.x, f1.centroid.y, r_map.get_pose().phi())
                    ) < r_map.get_pose().sqrDistanceTo(
                    mrpt::poses::CPose2D(f2.centroid.x, f2.centroid.y, r_map.get_pose().phi())
                    );
        });

        std::optional<std::deque<mrpt::math::TPoint2D>> travel_path;

        // Find the closest reachable
        for (auto& frontier : frontiers) {
            auto next_point = frontier.centroid;
            mrpt::poses::CPose2D next_roome_pose(next_point.x,next_point.y, r_map.get_pose().phi());
            travel_path = nav.find_path(r_map.get_grid_map(), r_map.get_pose(), next_roome_pose);
            if (!travel_path) {
                continue;
            } else {
                break;
            }

        }

        mrpt::utils::CImage img;
        r_map.get_grid_map().getAsImage(img, false, true);
        add_frontier_info(img, frontiers,r_map.get_grid_map());
        add_roome_position(img, r_map.get_pose(), r_map.get_grid_map());
        if (travel_path) {
            std::cout << "Path exists, Size: " << travel_path->size() << std::endl;
            add_path_nodes(img,*travel_path, r_map.get_grid_map());
        } else {
            std::cout << "Couldn't find a path, exiting." << std::endl;
        }

        // save scan
        output_name = "outputs/" + std::to_string(count++) + "_scan.jpg" ;
        img.saveToFile(output_name);

        if (!travel_path) break;



        int btw = 0;
        double curr_ang = 0.0;
        for (const auto& pt : *travel_path) {
            // Move to point, and get new pose:
            mrpt::poses::CPose2D new_pose(pt.x, pt.y, curr_ang);
            curr_ang =  get_new_ang(curr_ang);

            // Check if the point is reachable:
            auto next_point_path = nav.find_path(r_map.get_grid_map(), r_map.get_pose(), new_pose);
            // Start over if the next point is unreachable
            if (!next_point_path) break; 

            auto roome_pose = r_map.get_pose();

            mrpt::poses::CPose2D pose_delta = new_pose - roome_pose;
//            mrpt::poses::CPose2D pose_delta(new_pose.m_coords[0] - roome_pose.m_coords[0], 
//                                            new_pose.m_coords[1] - roome_pose.m_coords[1],
//                                            new_pose.phi() - roome_pose.phi());

            virtual_pose = new_pose;
            env.update_pose(virtual_pose);
            auto scan = env.scan();
            r_map.insert_observation(scan, pose_delta);  
            mrpt::poses::CPose2D potential_pose(pt.x,pt.y, r_map.get_pose().phi());
            r_map.get_grid_map().getAsImage(img, false, true);
            add_roome_position(img, r_map.get_pose(), r_map.get_grid_map());
            output_name = "outputs/" + std::to_string(count) + "_" + std::to_string(btw++) + "_scan.jpg";
            img.saveToFile(output_name);
        }

    }
    mrpt::utils::CImage img;
    r_map.get_grid_map().getAsImage(img, false, true);
    add_roome_position(img, r_map.get_pose(), r_map.get_grid_map());
    output_name = "outputs/final_scan.jpg";
    img.saveToFile(output_name);
    return 0;
}
