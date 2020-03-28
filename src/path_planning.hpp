#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <vector>

/*==============================================================================
                                Class definition
==============================================================================*/

/* MotionPlanner class definition */
class MotionPlanner {
public:
    /**
     * Car struct type definition containing car coordinates, yaw and speed.
     */
    struct Car {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    /**
     * Map struct type definition containing vectors of waypoints coordinates.
     */
    struct Map {
        std::vector<double> waypoints_x;
        std::vector<double> waypoints_y;
        std::vector<double> waypoints_s;
        std::vector<double> waypoints_dx;
        std::vector<double> waypoints_dy;
    };

    /**
     * Path struct type definition.
     */
    struct Path {
        std::vector<double> x;
        std::vector<double> y;
        double end_s;
        double end_d;
    };

    /** MotionPlanner constructor */
    MotionPlanner(double max_vel, double time_step = .02, int lane = 1,
                  int points_num = 50) :
        max_vel_(max_vel), time_step_(time_step), lane_(lane),
        points_num_(points_num) {}

    /** Function that returns the planned trajectory */
    void GenerateTrajectory(
                            Car &car, const Map &map,
                            const Path &previous_path,
                            const std::vector<std::vector<double>> &
                            sensor_fusion,
                            std::vector<double> &next_x_vals,
                            std::vector<double> &next_y_vals
                           );
private:
    double ref_vel_ = 0;
    double max_vel_;
    double time_step_;
    int lane_;
    int points_num_;
};

#endif  // PATH_PLANNING_HPP