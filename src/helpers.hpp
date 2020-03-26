#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <string>
#include <vector>

/*==============================================================================
                                Custom types
==============================================================================*/

struct Map {
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;
};

struct Car {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

/*==============================================================================
                                External functions
==============================================================================*/

std::string hasData(std::string s);

constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y);

std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);

#endif  // HELPERS_HPP