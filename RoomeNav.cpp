#include "RoomeNav.h"

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
