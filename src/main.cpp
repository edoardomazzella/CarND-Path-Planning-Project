#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.hpp"
#include "json.hpp"
#include "path_planning.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Instantiate MotionPlanner object
  /************/
  int lane = 1;
  double ref_vel = 49.5;
  /************/
  int traj_points_num = 50;
  double dist_inc = 0.3;
  MotionPlanner mp(traj_points_num, dist_inc);

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MotionPlanner::Map map;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map.waypoints_x.push_back(x);
    map.waypoints_y.push_back(y);
    map.waypoints_s.push_back(s);
    map.waypoints_dx.push_back(d_x);
    map.waypoints_dy.push_back(d_y);
  }

  h.onMessage([&mp,&map, &lane, ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          MotionPlanner::Car main_car = {
                                         j[1]["x"], j[1]["y"],
                                         j[1]["s"], j[1]["d"],
                                         j[1]["yaw"], j[1]["speed"]
                                        };

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same sideyy
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          /************/
          int prev_size = previous_path_x.size();

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // later we will interpolate these waypoints with a spline
          vector<double> ptsx, ptsy;

          // Reference x y yaw
          double ref_x = main_car.x;
          double ref_y = main_car.y;
          double ref_yaw = deg2rad(main_car.yaw);

          // If previous size is almost empty, use the car as starting reference
          if (prev_size < 2)
          {
            // Use two points that make the path tangent to the car
            double prev_car_x = main_car.x - cos(main_car.yaw);
            double prev_car_y = main_car.y - sin(main_car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(main_car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(main_car.y);
          }
          // Use the previous path's end point as starting reference
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
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
                                           main_car.s + (i + 1) * 30, (2 + 4 * lane), 
                                           map.waypoints_s, map.waypoints_x,     
                                           map.waypoints_y
                                          );
            ptsx.push_back(next_wp[0]);
            ptsx.push_back(next_wp[1]);
          }

          for (unsigned int i = 0; i < ptsx.size(); i++)
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

          /************/
          // Define the actual (x,y) points to be used for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /************/
          
          // Start with all the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired
          // reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2.0) + pow(target_y, 2.0));

          // Fill up the rest of the path after filling it with the previous point,
          // here we will always output 50 points
          double x_add_on = 0;
          for (unsigned int i = 1; i <= 50 - previous_path_x.size(); i++) 
          {
            double time_step = .02;
            double ref_vel_mps = ref_vel / 2.24; // conversion to meters/second
            double N = target_dist / (time_step * (ref_vel_mps));
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
          /************/

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //mp.GenerateTrajectory(main_car, map, next_x_vals, next_y_vals);

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}