#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <vector>

class MotionPlanner {
public:
    MotionPlanner(double dist_inc) : dist_inc(dist_inc) {}
    void GenerateTrajectory(double car_x, double car_y, double car_yaw,
                            std::vector<double> &next_x_vals, 
                            std::vector<double> &next_y_vals);
private:
    double dist_inc;
};

#endif  // PATH_PLANNING_HPP