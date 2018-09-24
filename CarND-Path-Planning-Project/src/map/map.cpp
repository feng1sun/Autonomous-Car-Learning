#include "map.h"
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

MAP::MAP(ifstream& in_map){
	string line;
	while (getline(in_map, line)) {
		istringstream iss(line);
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
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
    cout << "Create MAP!" << endl;
}

MAP::~MAP(){
	cout << "Delete MAP!" << endl;
}

vector<double> MAP::getWaypointsX(){
	return map_waypoints_x;
}

vector<double> MAP::getWaypointsY(){
	return map_waypoints_y;
}

vector<double> MAP::getWaypointsDX(){
	return map_waypoints_dx;
}

vector<double> MAP::getWaypointsDY(){
	return map_waypoints_dy;
}

vector<double> MAP::getWaypointsS(){
	return map_waypoints_s;
}