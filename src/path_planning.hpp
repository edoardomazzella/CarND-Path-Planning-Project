#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <vector>

/*==============================================================================
                                Class definition
==============================================================================*/

/* MotionPlanner class definition */
class MotionPlanner {
public:

    /** Car struct type definition containing car coordinates, yaw and speed. */
    struct Car {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
    };

    /**  Map struct type definition containing vectors of waypoints coordinates. */
    struct Map {
        std::vector<double> waypoints_x;
        std::vector<double> waypoints_y;
        std::vector<double> waypoints_s;
        std::vector<double> waypoints_dx;
        std::vector<double> waypoints_dy;
    };

    /** Path struct type definition. */
    struct Path {
        std::vector<double> x;
        std::vector<double> y;
        double end_s;
        double end_d;
    };

    /** MotionPlanner constructor */
    MotionPlanner(double max_vel, double trajectory_meters = 40,
                  int points_num = 70) :
        max_vel_(max_vel), time_step_(.02), lane_(1),
        trajectory_meters_(trajectory_meters), points_num_(points_num),
        too_close_(false), ref_vel_(0), lane_preference_(1),
        lane_preference_stability_(0) {}

    /** Function that returns the planned trajectory */
    void GenerateTrajectory(
                            Car &main_car, const Map &map,
                            const Path &previous_path,
                            const std::vector<std::vector<double>> &
                            sensor_fusion,
                            std::vector<double> &next_x_vals,
                            std::vector<double> &next_y_vals
                           );
private:
    double max_vel_; /** < Maximum desired velocity */
    double time_step_; /** < Time step */
    int lane_; /** < Selected lane */
    int points_num_; /** < Number of points in the generated trajectory */
    double trajectory_meters_; /** < Length of the trajectory */
    bool too_close_; /** < Is the car too close to the front vehicle? */
    double ref_vel_ ; /** < Reference velocity */
    int lane_preference_; /** < Lane preference */
    int lane_preference_stability_; /** < Lane preference stability */

/*============================================================================*/

    /** Function used to plan the high level behavior of the car */
    void PlanBehavior(
                      const Car &main_car,
                      const std::vector<std::vector<double>> &sensor_fusion,
                      int prev_size
                     );

    /** Function to get car information from sensor fusion data. */
    inline void GetPerceptedCarInformation_(
                                            const std::vector<double> &
                                            sensor_fusion_data, int prev_size,
                                            Car &other_car
                                           );

    /** Function to get car information from sensor fusion data. */
    inline int GetPerceptedCarLane_(double other_car_d)
    {
        return other_car_d / 4;
    }

    /** Set too_close_ to true if the car is too close */
    inline void UpdateTooClose_(const Car &main_car, const Car &car_in_lane);

    /** Get too_close_ value */
    inline bool isTooClose_() { return too_close_; }

    /** Function to adapt the velocity to the other vehicles speed */
    inline void AdaptVelocity_();

    /** Function to compute the lane choice cost */
    void UpdateLane_(
                     const Car &main_car,
                     const std::vector<Car> &center_lane_cars,
                     const std::vector<Car> &left_lane_cars,
                     const std::vector<Car> &right_lane_cars
                    );

    /** Function to compute the cost of keeping the same lane */
    double KeepLaneCost_(
                         const Car &main_car,
                         const std::vector<Car> &other_cars
                        );

    /** Function to compute the cost of changing lane */
    double ChangeLaneCost_(
                           const Car &main_car,
                           const std::vector<Car> &other_cars
                          );
};

#endif  // PATH_PLANNING_HPP