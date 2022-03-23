#include "RoomeNav.h"



RoomeNav::RoomeNav() {

}


std::optional<std::deque<mrpt::math::TPoint2D>> RoomeNav::find_path(
        const mrpt::maps::COccupancyGridMap2D& grid,
        const mrpt::poses::CPose2D start,
        const mrpt::poses::CPose2D finish) {
    return std::nullopt;
}
