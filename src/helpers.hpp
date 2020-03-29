#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <string>
#include <vector>

#define id_idx 0u
#define x_idx  1u
#define y_idx  2u
#define vx_idx 3u
#define vy_idx 4u
#define s_idx  5u
#define d_idx  6u

/*==============================================================================
                            Function Declarations
==============================================================================*/

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
std::string hasData(std::string s);

/** For converting back and forth between radians and degrees. */
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

/** Calculate distance between two points. */
double distance(double x1, double y1, double x2, double y2);

/** Calculate closest waypoint to current x, y position. */
int ClosestWaypoint(
                    double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y
                   );

/** Returns next waypoint of the closest waypoint. */
int NextWaypoint(
                 double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y
                );

/** Transform from Cartesian x,y coordinates to Frenet s,d coordinates. */
std::vector<double> getFrenet(
                              double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y
                             );

/** Transform from Frenet s,d coordinates to Cartesian x,y. */
std::vector<double> getXY(
                          double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y
                         );

#endif  // HELPERS_HPP