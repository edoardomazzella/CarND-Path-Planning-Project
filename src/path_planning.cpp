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
                                       Car &car, const Map &map,
                                       const Path &previous_path,
                                       const vector<vector<double>> &
                                       sensor_fusion,
                                       vector<double> &next_x_vals,
                                       vector<double> &next_y_vals
                                      )
{
    int prev_size = previous_path.x.size();
    std::cout << prev_size << std::endl;

    if (prev_size > 0) { car.s = previous_path.end_s; }

    bool too_close = false;
    // For each percepted car
    for (unsigned int i = 0; i < sensor_fusion.size(); i++)
    {
        Car other_car;

        // If other_car is in my lane
        other_car.d = sensor_fusion[i][6];
        if (
            other_car.d < (2 + 4 * lane_ + 2) &&
            other_car.d > (2 + 4 * lane_ - 2)
           )
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            other_car.speed = sqrt(pow(vx, 2.0) + pow(vy, 2.0));

            // Predict where the other car will be in the future based on speed
            other_car.s = sensor_fusion[i][5] + ((double) prev_size *
                          time_step_ * other_car.speed);

            // Check s values greater than mine and s gap
            if ((other_car.s > car.s) && (other_car.s - car.s) < 30)
            {
                too_close = true;
                if (lane_ > 0)
                {
                    lane_ = 0;
                }
            }
        }
    }

    if (true == too_close)
    {
        ref_vel_ -=.224;
    }
    else if (ref_vel_ < max_vel_)
    {
        ref_vel_ += .224;
    }
    else
    {
        // intentionally left empty
    }

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // later we will interpolate these waypoints with a spline
    vector<double> ptsx, ptsy;

    // Reference x y yaw
    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);

    // If previous size is almost empty, use the car as starting reference
    if (prev_size < 2)
    {
        // Use two points that make the path tangent to the car
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car.y);
    }
    // Else use the previous path's end point as starting reference
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

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    for (unsigned int i = 0; i < 3; ++i)
    {
        vector<double> next_wp = getXY(
                                        car.s + (i + 1) * 30, (2 + 4 * lane_),
                                        map.waypoints_s, map.waypoints_x,
                                        map.waypoints_y
                                      );
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }

    for (unsigned int i = 0; i < ptsx.size(); ++i)
    {
        // Shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Create a spline
    tk::spline s;

    // Set (x,y) points to the spline
    s.set_points(ptsx, ptsy);

    // Start with all the previous path points from last time
    for (int i = 0; i < prev_size; ++i)
    {
        next_x_vals.push_back(previous_path.x[i]);
        next_y_vals.push_back(previous_path.y[i]);
    }

    // Calculate how to break up spline points so that we travel at our desired
    // reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2.0) + pow(target_y, 2.0));

    // Fill up the rest of the path after filling it with the previous point,
    double x_add_on = 0;
    for (unsigned int i = 1; i <= points_num_ - prev_size; ++i)
    {
        double ref_vel_mps = ref_vel_ / 2.24; // conversion to meters/second
        double N = target_dist / (time_step_ * (ref_vel_mps));
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Convert car local coordinates back to global coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)) + ref_x;
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)) + ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

/*============================================================================*/