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

    /** MotionPlanner constructor */
    MotionPlanner(double ref_vel) : lane_(1), ref_vel_(ref_vel) {}

    /** Function that returns the planned trajectory */
    void GenerateTrajectory(
                            const Car &car, const Map &map,
                            const std::vector<double> &previous_path_x,
                            const std::vector<double> &previous_path_y,
                            double end_path_s, double end_path_d,
                            std::vector<double> &next_x_vals,
                            std::vector<double> &next_y_vals
                           );
private:
    int lane_;
    double ref_vel_;
};

#endif  // PATH_PLANNING_HPP