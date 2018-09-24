#ifndef __VEHICLE__
#define __VEHICLE__

#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <algorithm>
#include "../jerkSolver/jerkSolver.h"
#include "../utils/utils.h"
#include "../map/map.h"
#include "../trajectory/trajectory.h"

using namespace std;

class Vehicle {
public:
    Vehicle(MAP& map, ofstream& log);
    ~Vehicle();

    ofstream& log;
    /**
     * vehicle pose
     */
    typedef struct {
        double s;
        double d;
        double x;
        double y;
        double yaw;
        double speed;
        double sv;
        double sa;
        double dv;
        double da;
        // 6 parameters [a0 a1 a2 a3 a4 a5 a6]
        vector<double> Scoeffs;
        vector<double> Dcoeffs;
        vector<double> previous_path_x;
        vector<double> previous_path_y;
        double end_s;
        double end_d;
    } egoInfo;

    egoInfo ego;
    
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;


    vector<vector<double>> ego_sensor_fusion;

    vector<double> ego_next_x_vals;
    vector<double> ego_next_y_vals;


    void pipeline();
    void interpolate_waypoints(int ego_closest_point_idx, 
                    vector<double>& interpolate_waypoints_x,
                    vector<double>& interpolate_waypoints_y,
                    vector<double>& interpolate_waypoints_s);

    vector<Trajectory> generateTraj(vector<double>& map_x, 
                                    vector<double>& map_y, 
                                    vector<double>& map_s);

private:
};
#endif