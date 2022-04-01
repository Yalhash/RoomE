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
        const mrpt::poses::CPose2D& start,
        const mrpt::poses::CPose2D& finish) {
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

        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if ((i == 0 && j == 0)  
                   || (x + i < 0 || x + i >= num_cols) 
                   || (y + j < 0 || y + j >= num_rows)){
                  continue;
                }
                auto p = std::make_pair(x + i,y + j);
                if (seen.find(p) == seen.end()) {
                    seen.emplace(p);
                    que.push_back(p);
                } else {
                  continue;
                }
            }
        }
    }

    // Check roome sized area ,returns -1 if there is an object there and avg uncertainty value otherwise
    double find_area_uncertainty(const mrpt::maps::COccupancyGridMap2D& grid, 
                                 int x, int y, int num_rows, int num_cols) {
        int count = 0;
        double sum;
        // 12 wide square area average (To cover area of RoomE)
        for (int i = 0; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                //if out of range that means we cannot move here
                if ((x + i < 0 || x + i >= num_cols) || 
                    (y + j < 0 || y + j >= num_rows)) {
                  return -1;
                }
                auto prob = grid.getCell(x + i, y + j);
                // Cannot move here if there is a guaranteed object
                if (std::abs(prob) < 0.05){
                    return -1;
                }
                sum += prob;
                ++count;
            }
        }
        return sum / count;
    }
}

//probably want to keep track of direction we came from so we dont go there
std::optional<mrpt::math::TPoint2D> RoomeNav::find_destiny(
        const mrpt::maps::COccupancyGridMap2D& grid,
        const mrpt::poses::CPose2D& start) {
    // Get our starting point
    int curr_x =  start.m_coords[0];
    int curr_y =  start.m_coords[1];

    int num_rows = grid.getSizeX();
    int num_cols = grid.getSizeY();

    std::deque<std::pair<int,int>> que;

    //four starting points are x - 0.15
    std::set<std::pair<int,int>> seen;
    que.push_back(std::make_pair(curr_x, curr_y));
    while (!que.empty()) {
        auto p = que.front();
        que.pop_front();
        auto prob = find_area_uncertainty(grid, p.first, p.second, num_rows, num_cols);
        if (prob == -1) {
            continue;
        }
        else if (std::abs(prob - 0.5) < 0.05) {
            mrpt::math::TPoint2D ret_val(grid.idx2x(p.first), grid.idx2y(p.second));
            return std::optional<mrpt::math::TPoint2D>{ret_val};
        }
        get_nearby_points(que, seen, p.first, p.second, num_rows, num_cols);
    }
    
    return std::nullopt; // We are done...
}
//std::optional<mrpt::math::TPoint2D> RoomeNav::find_destiny(
//        const mrpt::maps::COccupancyGridMap2D& grid,
//        const mrpt::poses::CPose2D& start) {
//    // Get our starting point
//    int curr_x =  start.m_coords[0];
//    int curr_y =  start.m_coords[1];
//
//    int num_rows = grid.getSizeX();
//    int num_cols = grid.getSizeY();
//
//    double min_uncertainty = 1;
//
//    std::deque<std::pair<int,int,int>> que;
//
//
//    //forward
//    double area_uncertainty = find_area_uncertainty(grid, grid.x2idx(curr_x - 0.3) ,grid.y2idx(curr_y + 0.3) ,num_rows,num_cols);
//
//    if (area_uncertainty != -1 && area_uncertainty < min_uncertainty){
//        mrpt::math::TPoint2D ret_val(curr_x, curr_y + 0.3);
//        return  std::optional<mrpt::math::TPoint2D>{};
//    }
//
//    //left 
//    area_uncertainty = find_area_uncertainty(grid, grid.x2idx(curr_x - 0.3) ,grid.y2idx(curr_y + 0.3) ,num_rows,num_cols);
//
//    if (area_uncertainty != -1){
//        mrpt::math::TPoint2D ret_val(curr_x, curr_y + );
//        return  std::optional<mrpt::math::TPoint2D>{ret_val};
//    }
//    
//
//    //four starting points are x - 0.15
//    que.push_back()
//
//    std::set<std::pair<int,int>> seen;
//    que.push_back(std::make_pair(x_ind, y_ind));
//
//
//
//    
//    return std::nullopt; // We are done...
//}
