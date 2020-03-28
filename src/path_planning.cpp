#include "path_planning.hpp"
#include "helpers.hpp"
#include <math.h>

using std::vector;

/*==============================================================================
                        Public Member Function Definitions
==============================================================================*/

void MotionPlanner::GenerateTrajectory(
                                       const Car &car, const Map &map,
                                       vector<double> &next_x_vals,
                                       vector<double> &next_y_vals
                                      )
{
    for (unsigned int i = 0; i < traj_points_num_; ++i) {
        double next_s = car.s + (i+1) * dist_inc_;
        double next_d = 6;
        vector<double> xy = getXY(next_s, next_d, map.waypoints_s,
                                  map.waypoints_x, map.waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
}

/*============================================================================*/