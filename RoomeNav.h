#ifndef ROOME_NAV_H
#define ROOME_NAV_H
// Standard library
#include <optional>
#include <deque>

// MRPT
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/poses/CPose2D.h>

class RoomeNav {
public:
    RoomeNav();
    std::optional<std::deque<mrpt::math::TPoint2D>> find_path(
            const mrpt::maps::COccupancyGridMap2D& grid,
            const mrpt::poses::CPose2D start,
            const mrpt::poses::CPose2D finish);

    std::optional<mrpt::math::TPoint2D> find_destiny(
            const mrpt::maps::COccupancyGridMap2D& grid,
            const mrpt::poses::CPose2D start);
private:
    mrpt::nav::PlannerSimple2D planner;
};
#endif
