#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include "helpers.hpp"
#include <vector>

/*==============================================================================
                                Class definition
==============================================================================*/

class MotionPlanner {
public:
    MotionPlanner(int traj_points_num, double dist_inc) :
        traj_points_num_(traj_points_num), dist_inc_(dist_inc) {}

    void GenerateTrajectory(const Car &car, const Map &map,
                            std::vector<double> &next_x_vals,
                            std::vector<double> &next_y_vals);
private:
    int traj_points_num_;
    double dist_inc_;
};

#endif  // PATH_PLANNING_HPP