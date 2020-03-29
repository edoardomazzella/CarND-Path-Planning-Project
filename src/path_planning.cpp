#include "path_planning.hpp"
#include "helpers.hpp"
#include "spline.h"
#include <math.h>
#include <iostream>
#include <utility>

using std::vector;

/*==============================================================================
                        Public Member Function Definitions
==============================================================================*/

void MotionPlanner::GenerateTrajectory(
                                       Car &main_car, const Map &map,
                                       const Path &previous_path,
                                       const vector<vector<double>> &
                                       sensor_fusion,
                                       vector<double> &next_x_vals,
                                       vector<double> &next_y_vals
                                      )
{
    too_close_ = false;
    int prev_size = previous_path.x.size();
    if (prev_size > 0) { main_car.s = previous_path.end_s; }

    // Plan velocity and choose lane
    PlanBehavior(main_car, sensor_fusion, prev_size);

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // later we will interpolate these waypoints with a spline.
    vector<double> ptsx, ptsy;

    // Reference x, y and yaw
    double ref_x = main_car.x;
    double ref_y = main_car.y;
    double ref_yaw = deg2rad(main_car.yaw);

    // If previous size is almost empty, use the car as starting reference.
    if (prev_size < 2)
    {
        // Use two points that make the path tangent to the car.
        double prev_car_x = main_car.x - cos(main_car.yaw);
        double prev_car_y = main_car.y - sin(main_car.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(main_car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(main_car.y);
    }
    // Else use the previous path's end point as starting reference.
    else
    {
        ref_x = previous_path.x[prev_size - 1];
        ref_y = previous_path.y[prev_size - 1];
        double ref_x_prev = previous_path.x[prev_size - 2];
        double ref_y_prev = previous_path.y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference.
    for (unsigned int i = 0; i < 3; ++i)
    {
        vector<double> next_wp = getXY(
                                       main_car.s + (i + 1) *
                                       trajectory_meters_,
                                       (2 + 4 * lane_),
                                       map.waypoints_s, map.waypoints_x,
                                       map.waypoints_y
                                      );
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    for (unsigned int i = 0; i < ptsx.size(); ++i)
    {
        // Shift car reference angle to 0 degrees.
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Create a spline.
    tk::spline s;

    // Set (x,y) points to the spline.
    s.set_points(ptsx, ptsy);

    // Start with all the previous path points from last time.
    for (int i = 0; i < prev_size; ++i)
    {
        next_x_vals.push_back(previous_path.x[i]);
        next_y_vals.push_back(previous_path.y[i]);
    }

    // Calculate how to break up spline points so that we travel at our desired
    // reference velocity.
    double target_x = trajectory_meters_;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2.0) + pow(target_y, 2.0));

    // Fill up the rest of the path.
    double x_add_on = 0;
    for (unsigned int i = 1; i <= points_num_ - prev_size; ++i)
    {
        double ref_vel_mps = ref_vel_ / 2.24; // conversion from mph/h to m/s.
        double N = target_dist / (time_step_ * (ref_vel_mps));
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Convert car local coordinates back to global coordinates.
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

/*==============================================================================
                        Private Member Function Definitions
==============================================================================*/

void MotionPlanner::PlanBehavior(
                                 const Car &main_car,
                                 const vector<vector<double>> &sensor_fusion,
                                 int prev_size
                                )
{
    vector<Car> center_lane_cars;
    vector<Car> left_lane_cars;
    vector<Car> right_lane_cars;
    // For each percepted car.
    for (unsigned int i = 0; i < sensor_fusion.size(); ++i)
    {
        Car other_car;

        // Obtain coordinates and speed
        GetPerceptedCarInformation_(sensor_fusion[i], prev_size, other_car);

        // Assign car to a lane
        int other_car_lane = GetPerceptedCarLane_(other_car.d);
        switch(other_car_lane)
        {
            case 0:
                left_lane_cars.push_back(other_car);
                break;
            case 1:
                center_lane_cars.push_back(other_car);
                break;
            case 2:
                right_lane_cars.push_back(other_car);
                break;
            default:
                break;
        }

        // Remember if the car is too close to a vehicle to properly set the velocity.
        if (other_car_lane == lane_)
        {
            UpdateTooClose_(main_car, other_car);
        }
    }

    // Adapt the velocity based on the other vehicles observations.
    AdaptVelocity_();

    // Update the lane
    UpdateLane_(main_car, center_lane_cars, left_lane_cars, right_lane_cars);
}

/*============================================================================*/

inline void MotionPlanner::GetPerceptedCarInformation_(
                                                       const vector<double>
                                                       &sensor_fusion_data,
                                                       int prev_size,
                                                       Car &other_car
                                                      )
{
     // Other car global coordinates
    other_car.x = sensor_fusion_data[x_idx];
    other_car.y = sensor_fusion_data[y_idx];
    // Other car speed
    double vx = sensor_fusion_data[vx_idx];
    double vy = sensor_fusion_data[vy_idx];
    other_car.speed = sqrt(pow(vx, 2.0) + pow(vy, 2.0));
    // Predict where the other car will be in the future based on speed.
    other_car.s = sensor_fusion_data[s_idx] + ((double) prev_size *
                  time_step_ * other_car.speed);
    // Other car distance from reference trajectory
    other_car.d = sensor_fusion_data[d_idx];
}

/*============================================================================*/

inline void MotionPlanner::UpdateTooClose_(
                                           const Car &main_car,
                                           const Car &car_in_lane
                                          )
{
    // Check s values greater than mine and s gap.
    if (
        (car_in_lane.s > main_car.s) &&
        ((car_in_lane.s - main_car.s) < trajectory_meters_)
        )
    {
        too_close_ = true;
    }
}

/*============================================================================*/

inline void MotionPlanner::AdaptVelocity_()
{
    if (true == isTooClose_())
    {
        ref_vel_ -=.224;
    }
    else if (ref_vel_ < max_vel_)
    {
        ref_vel_ += .224;
    }
    else
    {
        // Intentionally left empty.
    }
}

/*============================================================================*/

void MotionPlanner::UpdateLane_(
                                const Car &main_car,
                                const std::vector<Car> &center_lane_cars,
                                const std::vector<Car> &left_lane_cars,
                                const std::vector<Car> &right_lane_cars
                               )
{
    vector< std::vector<Car> > lane_cars =
        { left_lane_cars, center_lane_cars, right_lane_cars };

    // Initialize the best lane and its cost with the current lane values
    std::pair < int, double > best_lane_and_cost =
        { lane_, KeepLaneCost_(main_car, lane_cars[lane_]) };

    std::cout <<  "My lane cost = " << best_lane_and_cost.second << std::endl;
    for (int i = 0; i < 3; ++i)
    {
        // Consider only the lanes next to you
        if (fabs(i - lane_) == 1)
        {
            // Compute the cost of changing lane
            double cost = ChangeLaneCost_(main_car, lane_cars[i]);
            // If the cost is minimum store the id and the cost
            if (cost < best_lane_and_cost.second)
            {
                best_lane_and_cost = { i, cost };
            }
            std::cout <<  "Lane[" << i << "] cost = " << cost << std::endl;
        }
    }

    // Compare the current preference with the previous one and update the
    // stability of the preference
    if (best_lane_and_cost.first == lane_preference_)
    {
        if(lane_preference_stability_ + 1 <= 10)
        {
            lane_preference_stability_++;
        }
    }
    else
    {
        lane_preference_ = best_lane_and_cost.first;
        lane_preference_stability_ = 0;
    }
    // Update the lane if the stability is >= 10
    if (lane_ != lane_preference_ && lane_preference_stability_ >= 10)
    {
        lane_ = lane_preference_;
    }
    std::cout << "Lane preference = " << lane_preference_ << std::endl;
    std::cout << "Chosen lane = " << lane_ << std::endl << std::endl;
}

/*============================================================================*/

double MotionPlanner::KeepLaneCost_(
                                    const Car &main_car,
                                    const std::vector<Car> &other_cars
                                   )
{
    double lane_cost = 0;

    // Increment the cost of keeping the lane if there is a slow vehicle
    // in front of us
    for (auto other_car : other_cars)
    {
        if (
            (main_car.s < other_car.s) &&
            ((other_car.s - main_car.s) < trajectory_meters_) &&
            (other_car.speed <= main_car.speed)
           )
        {
            // cost increment based on the other vehicle velocity
            lane_cost += (((main_car.speed - other_car.speed) / 2.24) /
                            max_vel_) / 2;
        }
    }

    return lane_cost;
}

/*============================================================================*/

double MotionPlanner::ChangeLaneCost_(
                                      const Car &main_car,
                                      const std::vector<Car> &other_cars
                                     )
{
    // Changing lane is penalized by default
    double lane_cost = 0.01;

    for (auto other_car : other_cars)
    {
        // If there is a vehicle behind us increment the cost
        if (
            (main_car.s >= other_car.s) &&
            ((main_car.s - other_car.s) < trajectory_meters_)
           )
        {
            // cost increment (negative rvalue) based on the other vehicle
            // position.
            // Examples:
            // trajectory_meters_ of distance -> cost increment = 0.0
            // trajectory_meters_/2 of distance -> cost increment = 0.25
            // 0m of distance -> cost increment = 0.5
            lane_cost -= (((main_car.s - other_car.s) - trajectory_meters_) /
                            trajectory_meters_) / 2;
            // cost increment/decrement based on the other vehicle velocity
            // Examples:
            // max_vel_ of difference -> cost increment/decrement = 0.5
            // max_vel_/2 of distance -> cost increment/decrement = 0.25
            // 0 speed difference -> cost increment/decement = 0.0
            lane_cost += (((other_car.speed - main_car.speed) / 2.24) /
                            max_vel_) / 2;
        }
        // If there is a vehicle in front of us increment the cost
        if (
            (main_car.s < other_car.s) &&
            ((other_car.s - main_car.s) < trajectory_meters_)
           )
        {
            // cost increment (negative rvalue) based on the other vehicle
            // position.
            lane_cost -= (((other_car.s - main_car.s) - trajectory_meters_) /
                            trajectory_meters_) / 2;
            // cost increment/decrement based on the other vehicle velocity
            lane_cost += (((main_car.speed - other_car.speed) / 2.24) /
                            max_vel_) / 2;
        }
    }

    // return a value that is always > 0.01 for changing lane
    return (lane_cost > 0.01)? lane_cost : 0.01;
}

/*============================================================================*/
