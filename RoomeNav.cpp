#include "RoomeNav.h"
#include <deque>
#include <set>
#include <utility> // pair
#include <cmath>
#include <tuple>


#define FREE_PROB 0.65f // Probably unoccupied
RoomeNav::RoomeNav() {
    planner.robotRadius = 0.2; // RoomE is about 26 cm, rounding to 30 for safety.
    planner.occupancyThreshold = 0.49f; // Allow Roome to tranverse some unknown areas
}

std::optional<std::deque<mrpt::math::TPoint2D>> RoomeNav::find_path(
        const mrpt::maps::COccupancyGridMap2D& grid,
        const mrpt::poses::CPose2D& start,
        const mrpt::poses::CPose2D& finish) {
    std::deque<mrpt::math::TPoint2D> detailed_path;
    std::deque<mrpt::math::TPoint2D> result_path;
    bool notFound;
    planner.computePath(grid, start, finish, detailed_path, notFound);

    if (notFound) return std::nullopt;

    // No need to shorten the path
    if (detailed_path.size() <= 2) {
        return std::optional<std::deque<mrpt::math::TPoint2D>>{result_path};
    }


    // Cut down on the number of points by only storing the points which are not straight lines from their previous point
    // Assume that the robot is about at the start point
    //mrpt::math::TPoint2D start_point(detailed_path[0]);
    //mrpt::math::TPoint2D curr_point(detailed_path[1]);
    //double slope = 0;
    //for (int i = 2; i < detailed_path.size(); ++i) {
        //// If the points are colinear we can skip past them, otherwise a new line is being added
        //if (abs((start_point.x - curr_point.x)*(curr_point.y - detailed_path[i].y)
        //- (curr_point.x - detailed_path[i].x)*(start_point.y - curr_point.y)) > 0.01) {
            //result_path.push_back(detailed_path[i-1]);
            //start_point = detailed_path[i-1];
            //curr_point = detailed_path[i];
        //} 
       // 
    //}
//
    //result_path.push_back(detailed_path.back());


    return std::optional<std::deque<mrpt::math::TPoint2D>>{detailed_path};
    //return std::optional<std::deque<mrpt::math::TPoint2D>>{result_path};
}

// Frontier Helper functions!
namespace {
    bool is_in_bounds(const std::pair<int, int>& p, const mrpt::maps::COccupancyGridMap2D& grid) {
        return p.first  >= grid.getXMin() && p.first  <= grid.getXMax() 
            && p.second >= grid.getYMin() && p.second <= grid.getYMax();
    }

    std::vector<std::pair<int,int>> nbhood(const std::pair<int, int>& p, const mrpt::maps::COccupancyGridMap2D& grid) {
        std::vector<std::pair<int,int>> result;
        int x = p.first;
        int y = p.second;
        // i,j = -1, 0, 1
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                auto new_p = std::make_pair(x + i, y + j);
                if ((i == 0 && j == 0) && is_in_bounds(new_p, grid)) continue;
                result.emplace_back(new_p);
            }
        }

        return result;
    }


    bool is_frontier_cell(const std::pair<int,int>& pt, 
        const mrpt::maps::COccupancyGridMap2D& grid,
        const std::set<std::pair<int,int>>& frontier_pts) {

        //If this is a part of another frontier, or it is not unknown, it is not a point
        /* std::cout << "first: " << (frontier_pts.find(pt) != frontier_pts.end()) << " second: " << grid.getCell(pt.first, pt.second) << std::endl; */

        if (frontier_pts.find(pt) != frontier_pts.end() || std::abs(grid.getCell(pt.first, pt.second)- 0.5) > 0.05) {
            return false;
        }

        // It also must be connected to a known unoccupied spot
        for (const auto& neigh_pts : nbhood(pt, grid)) {
            if (grid.getCell(neigh_pts.first, neigh_pts.second) > FREE_PROB) {
                return true;
            }
        }
        return false;
    }


    double find_distance(const std::pair<int, int>& p1, const std::pair<int, int>& p2, const mrpt::maps::COccupancyGridMap2D& grid) {
        double x1 = grid.idx2x(p1.first);
        double y1 = grid.idx2y(p1.second);
        double x2 = grid.idx2x(p2.first);
        double y2 = grid.idx2y(p2.second);

        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }


RoomeNav::Frontier make_frontier(const std::pair<int,int>& front_pt, 
        const std::pair<int,int>& robo_pt,
        const mrpt::maps::COccupancyGridMap2D& grid,
            std::set<std::pair<int,int>> & frontier_pts) {

        RoomeNav::Frontier result;
        result.centroid.x = 0;
        result.centroid.y = 0;

        std::deque<std::pair<int,int>> que;

        que.push_back(front_pt);
        while(!que.empty()) {
            auto p = que.front();
            que.pop_front();

            for (auto neigh_pt : nbhood(p, grid)) {

                if (is_frontier_cell(neigh_pt, grid, frontier_pts)) {
                    std::pair<int,int> new_pt(neigh_pt.first, neigh_pt.second);
                    frontier_pts.emplace(neigh_pt);
                    result.pts.push_back(neigh_pt);
                    auto dist = find_distance(robo_pt, neigh_pt, grid);
                    que.push_back(neigh_pt);
                    result.centroid.x += grid.idx2x(neigh_pt.first);
                    result.centroid.y += grid.idx2y(neigh_pt.second);

                    if (result.min_dist == -1 || result.min_dist > dist) {
                        result.min_dist = dist;
                        result.closest = mrpt::math::TPoint2D(grid.idx2x(neigh_pt.first), grid.idx2y(neigh_pt.second));
                    }
                }
            }
        }
        result.centroid.x /= result.pts.size();
        result.centroid.y /= result.pts.size();
        return result;
    }

} // End of helper functions

std::vector<RoomeNav::Frontier> RoomeNav::find_frontiers(
        const mrpt::maps::COccupancyGridMap2D& grid,
        const mrpt::poses::CPose2D& start) {

    // This starts as 200, and decreases by 50 until a new frontier is found, or 0 is reached
    unsigned int min_front_size = 200; // Don't pay attention to noise

    std::vector<RoomeNav::Frontier> frontiers;
    auto start_p = std::make_pair(grid.x2idx(start.m_coords[0]), grid.y2idx(start.m_coords[1]));

    while (min_front_size != 0) {
        std::deque<std::pair<int,int>> que;
        std::set<std::pair<int,int>> seen;
        std::set<std::pair<int,int>> frontier_pts;

        que.push_back(start_p);
        while (!que.empty()) {
            auto p = que.front();
            que.pop_front();
            for (const auto neigh_pt : nbhood(p, grid)) {
                // Add all free unseen cells 
                if (grid.getCell(neigh_pt.first, neigh_pt.second) > FREE_PROB && seen.find(neigh_pt) == seen.end()) {
                    seen.insert(neigh_pt);
                    que.push_back(neigh_pt);
                } else if (is_frontier_cell(neigh_pt, grid, frontier_pts)) {
                    frontier_pts.insert(neigh_pt);
                    auto new_front = make_frontier(neigh_pt, start_p, grid, frontier_pts);
                    if (new_front.pts.size() > min_front_size) {
                        frontiers.push_back(new_front);
                    }
                }
            }
        }
        if (frontiers.size() != 0) {
            break;
        } else {
            min_front_size -= 50;
            std::cout << "Decreasing the minimum frontier size by 50, looking for frontiers of size: " << min_front_size << std::endl;
        }
    }

    return frontiers;
}
