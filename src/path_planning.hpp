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
    MotionPlanner(int traj_points_num, double dist_inc) :
        traj_points_num_(traj_points_num), dist_inc_(dist_inc) {}

    /** Function that returns the planned trajectory */
    void GenerateTrajectory(
                            const Car &car, const Map &map,
                            std::vector<double> &next_x_vals,
                            std::vector<double> &next_y_vals
                           );
private:
    /** Number of points of the generated trajectory */
    int traj_points_num_;
    /** Distance increment */
    double dist_inc_;
};

#endif  // PATH_PLANNING_HPP