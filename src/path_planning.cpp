#include "path_planning.hpp"
#include "helpers.hpp"
#include <math.h>

using std::vector;

void MotionPlanner::GenerateTrajectory(double car_x, double car_y, 
    double car_yaw, vector<double> &next_x_vals, 
    vector<double> &next_y_vals) {
    for (int i = 0; i < 50; ++i) {
        next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
        next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
}