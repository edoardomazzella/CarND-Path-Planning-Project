#include "path_planning.hpp"
#include "helpers.hpp"
#include "spline.h"
#include <math.h>
#include <iostream>

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

    vector<Car> center_lane_cars;
    vector<Car> left_lane_cars;
    vector<Car> right_lane_cars;
    // For each percepted car.
    for (unsigned int i = 0; i < sensor_fusion.size(); i++)
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
    UpdateLane_();

    /* Trajectory Generation */

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

void MotionPlanner::AdaptVelocity_()
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