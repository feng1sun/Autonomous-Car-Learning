
#ifndef __HELP__
#define __HELP__

#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include <string>

using namespace std;

double LaneToD(int lane);

int DToLane(double d);
// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mph2mps(double x) { return x * 0.44704; }

string hasData(string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, vector<double> eval_at_x);

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y, double interval, int output_size);

double logistic(double x);

#endif