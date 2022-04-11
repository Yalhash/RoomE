// standard libs
#include <iostream>
#include <string>
#include <cctype>
#include <algorithm>
// MRPT includes
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/math/utils.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/utils/CImage.h>
// Lidar includes
#include "CYdLidar.h"
// local includes
#include "ArduinoSerial.h"
#include "Lidar_MRPT.h"
#include "RoomeMap.h"
#include "RoomeNav.h"
#include "DriveTrain.h"


void add_frontier_info(mrpt::utils::CImage& img, 
                       const std::vector<RoomeNav::Frontier>& fronts,
                       const mrpt::maps::COccupancyGridMap2D& grid);

void add_roome_position(mrpt::utils::CImage& img, 
                        const mrpt::poses::CPose2D& roome_pose,
                        const mrpt::maps::COccupancyGridMap2D& grid);

void add_path_nodes(mrpt::utils::CImage& img,
        const std::deque<mrpt::math::TPoint2D>& travel_path,
        const mrpt::maps::COccupancyGridMap2D& grid);

RoomeNav::Frontier find_nearest_frontier(const std::vector<RoomeNav::Frontier>& frontiers,
                                         const RoomeMap& r_map);

int main() {
    Lidar_MRPT lidar;
    if (!lidar.initialize()) {
        printf("Lidar initialization failed! Exiting.");
    }
    RoomeMap r_map;
    RoomeNav nav;
    DriveTrain d_train;
    std::string output_name;

    // Insert the base observation
    auto scan1 = lidar.scan();
    // 0 Difference scan;
    r_map.initial_observation(scan1);
    int count = 0;

	mrpt::utils::CImage img;
    while (true) {
        // Look for frontiers to move to
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

        // save scan
        r_map.get_grid_map().getAsImage(img, false, true);
        if (frontiers.size() != 1) 
            add_frontier_info(img, frontiers,r_map.get_grid_map());
        add_roome_position(img, r_map.get_pose(), r_map.get_grid_map());
        if (travel_path) {
            std::cout << "Path exists, Size: " << travel_path->size() << std::endl;
            add_path_nodes(img,*travel_path, r_map.get_grid_map());
        } else {
            std::cout << "Couldn't find a path to any fontier, exiting." << std::endl;
        }
        output_name = "outputs/" + std::to_string(count++) + "_scan.jpg" ;
        img.saveToFile(output_name);

        // Stop if there is no path to any frontiers (We are done)
        if (!travel_path) break;

        mrpt::poses::CPose2D pose_delta;
        for (const auto& pt : *travel_path) {

            // Check if the point is reachable:
            mrpt::poses::CPose2D potential_pose(pt.x,pt.y, r_map.get_pose().phi());
            auto next_point_path = nav.find_path(r_map.get_grid_map(), r_map.get_pose(), potential_pose);

            // Start over if the next point is unreachable
            if (!next_point_path) break; 

            // Move to point along the path,
            pose_delta = d_train.calculate_and_turn(r_map.get_pose(), pt);

            // scan and re-localize,
            auto scan = lidar.scan();
            r_map.insert_observation(scan, pose_delta);  

            // Move forward to scan again
            pose_delta = d_train.post_scan_drive();
            scan = lidar.scan();
            r_map.insert_observation(scan, pose_delta);  
        }
        
    }
    // Save the final scan
    r_map.get_grid_map().getAsImage(img, false, true);
    add_roome_position(img, r_map.get_pose(), r_map.get_grid_map());
    output_name = "outputs/final_scan.jpg";
    img.saveToFile(output_name);
    return 0;
}

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
