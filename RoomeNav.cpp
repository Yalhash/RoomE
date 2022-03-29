#include "RoomeNav.h"
#include <deque>
#include <set>
#include <utility> // pair
#include <cmath>

RoomeNav::RoomeNav() {
    planner.robotRadius = 0.30f; // RoomE is about 26 cm, rounding to 30 for safety.
}

std::optional<std::deque<mrpt::math::TPoint2D>> RoomeNav::find_path(
        const mrpt::maps::COccupancyGridMap2D& grid,
        const mrpt::poses::CPose2D start,
        const mrpt::poses::CPose2D finish) {
    std::deque<mrpt::math::TPoint2D> result_path;
    bool notFound;
    planner.computePath(grid, start, finish, result_path, notFound);

    if (notFound) return std::nullopt;

    return std::optional<std::deque<mrpt::math::TPoint2D>>{result_path};
}

namespace {
    void get_nearby_points(std::deque<std::pair<int,int>>& que,
                std::set<std::pair<int,int>>& seen,
                int x, int y, int num_rows, int num_cols) {
        for (int i = -1; i <= 1 ; ++i) {
            for (int j = -1; j <= 1 && y + j >= 0 && y + j < num_rows; ++j) {
                if ((i == 0 && j == 0) 
                  ||(x + i < 0 || x + i >= num_cols)
                  ||(y + j < 0 || y + j >= num_rows)) continue;

                auto p = std::make_pair(x + i,y + j);
                if (seen.find(p) == seen.end()) {
                    seen.emplace(p);
                    que.push_back(p);
                }
            }
        }
    }
}


std::optional<mrpt::math::TPoint2D> RoomeNav::find_destiny(
        const mrpt::maps::COccupancyGridMap2D& grid,
        const mrpt::poses::CPose2D start) {
    // Get our starting point
    int x_ind =  grid.x2idx(start.m_coords[0]);
    int y_ind =  grid.y2idx(start.m_coords[1]);
    std::deque<std::pair<int,int>> que;
    std::set<std::pair<int,int>> seen;
    que.push_back(std::make_pair(x_ind, y_ind));
    while (!que.empty()) {
        auto p = que.front();
        que.pop_front();
        auto prob = grid.getCell(p.first, p.second);
        // If we have found an uncertain value, return that point
        if (std::abs(prob - 0.5) < 0.05) { 
            mrpt::math::TPoint2D ret_val(grid.idx2x(p.first), grid.idx2y(p.second));
            return  std::optional<mrpt::math::TPoint2D>{ret_val};
        // If we have found a wall, dont go further
        } else if (std::abs(prob - 0) < 0.05){
            continue;
        }
        get_nearby_points(que, seen, p.first, p.second, 
                          grid.getSizeX(), grid.getSizeY());
    }
    // BFS out from where we are
    
    return std::nullopt; // We are done...
}
