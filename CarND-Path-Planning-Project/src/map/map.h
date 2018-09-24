#pragma once

#include <vector>
#include <fstream>
#include <sstream>

using namespace std;

class MAP{
public:
    MAP(ifstream& in_map);
    ~MAP();
    vector<double> getWaypointsX();
    vector<double> getWaypointsY();
    vector<double> getWaypointsS();
    vector<double> getWaypointsDX();
    vector<double> getWaypointsDY();
    
private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};