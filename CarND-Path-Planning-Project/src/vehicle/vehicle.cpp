#include "vehicle.h"
#include <iostream>
#include <set>
#include <cmath>
#include <fstream>
#include "../jerkSolver/jerkSolver.h"
#include "../map/map.h"
#include "../utils/utils.h"
#include "../utils/spline.h"
#include "../prediction/prediction.h"
#include "../trajectory/trajectory.h"

#define DEBUG

#define NUM_WAYPOINTS_BEHIND 3
#define NUM_WAYPOINTS_AHEAD 7

using namespace std;

Vehicle::Vehicle(MAP& map, ofstream& log) : log(log){
    map_waypoints_s = map.getWaypointsS();
    map_waypoints_x = map.getWaypointsX();
    map_waypoints_y = map.getWaypointsY();
    map_waypoints_dx = map.getWaypointsDX();
    map_waypoints_dy = map.getWaypointsDX();

    ego.Scoeffs = vector<double> {0, 0, 0, 0, 0, 0};
    ego.Dcoeffs = vector<double> {0, 0, 0, 0, 0, 0};
}

Vehicle::~Vehicle(){
}

void Vehicle::pipeline(){
    vector<double> interpolate_waypoints_x;
    vector<double> interpolate_waypoints_y;
    vector<double> interpolate_waypoints_s;
    
    int closest_point_idx = ClosestWaypoint(ego.x, ego.y, map_waypoints_x, map_waypoints_y);
    interpolate_waypoints(closest_point_idx, 
                        interpolate_waypoints_x, 
                        interpolate_waypoints_y, 
                        interpolate_waypoints_s);

#ifndef DEBUG
    // The reason to add 1 to next_wappoint_idx is that 
    // to avoid next_wappoint_idx = closest_point_idx which could lead to stop ego car.
    closest_point_idx = ClosestWaypoint(ego.x, ego.y, interpolate_waypoints_x, interpolate_waypoints_y);
    int next_wappoint_idx = NextWaypoint(ego.x, ego.y, ego.yaw, interpolate_waypoints_x, interpolate_waypoints_y) + 20;
    
    double closest_x = interpolate_waypoints_x[closest_point_idx];
    double closest_y = interpolate_waypoints_y[closest_point_idx];
    double closest_s = interpolate_waypoints_s[closest_point_idx];

    double next_x = interpolate_waypoints_x[next_wappoint_idx];
    double next_y = interpolate_waypoints_y[next_wappoint_idx];
    double next_s = interpolate_waypoints_s[next_wappoint_idx];

    double closest_point_dis = distance(ego.x, ego.y, closest_x, closest_y);
    double next_point_dis = distance(ego.x, ego.y, next_x, next_y);

    ego.sv = jerkEvalutate(ego.Scoeffs, 1); // s velocity
    ego.sa = jerkEvalutate(ego.Scoeffs, 2); // s acc

    ego.dv = jerkEvalutate(ego.Dcoeffs, 1); // d velocity
    ego.da = jerkEvalutate(ego.Dcoeffs, 2); // d acc

    log << "ego\t\t x:" << ego.x << " y:" << ego.y 
    << " s:" << ego.s << " d:" << ego.d 
    << " v:" << ego.speed << " sv:" << ego.sv << " sa:" << ego.sa
    << " dv:" << ego.dv << " da:" << ego.da
    << endl
    << "closest\t idx:" << closest_point_idx
    << " distance:" << closest_point_dis
    << " x:" <<  closest_x
    << " y:" << closest_y
    << " s:" << closest_s
    << endl
    << "next\t idx:" << next_wappoint_idx
    << " distance:" << next_point_dis
    << " x:" << next_x
    << " y:" << next_y
    << " s:" << next_s
    << endl;

    vector<double> start_pos_s {ego.s, ego.speed, ego.sa};
    vector<double> start_pos_d {6, ego.dv, ego.da};
    vector<double> end_pos_s {next_s, mph2mps(49), 9};
    vector<double> end_pos_d {6, 0, 0};

    // calculate minimization jerk coefficients
    ego.Scoeffs = jerkCoeffsSolve(start_pos_s, end_pos_s, 1);
    ego.Dcoeffs = jerkCoeffsSolve(start_pos_d, end_pos_d, 1);
    // jerkMinTraj {S, D}
    vector<vector<double>> jerkMinTraj = jerkInterpolation(ego.Scoeffs, ego.Dcoeffs);
    vector<double> S = jerkMinTraj[0];
    vector<double> D = jerkMinTraj[1];

    vector<double> X;
    vector<double> Y;

    for(int i = 0; i < S.size(); i++){
        double s = S[i];
        double d = D[i];
        vector<double> posXY = getXY(s, d, interpolate_waypoints_s, interpolate_waypoints_x, interpolate_waypoints_y);
        X.push_back(posXY[0]);
        Y.push_back(posXY[1]);
    }

    tk::spline smooth;
    smooth.set_points(X, Y);
    
    vector<double> next_x_trajectory;
    vector<double> next_y_trajectory;

    log << "New Trajectory: " << endl;
    double step = mph2mps(49) / 50;
    for(int i = 0; i < 50; i++){
        double x = ego.x + step * i;
        double y = smooth(x);
        next_x_trajectory.push_back(x);
        next_y_trajectory.push_back(y);
        log << "[" << x << "," << y << "] ";
    }
    ego_next_x_vals.clear();
    ego_next_y_vals.clear();
    ego_next_x_vals = next_x_trajectory;
    ego_next_y_vals = next_y_trajectory;

#endif
    // **************************************
    // ********* Test genTray ***************
#ifdef DEBUG
    ego.sv = jerkEvalutate(ego.Scoeffs, 1); // s velocity
    ego.sa = jerkEvalutate(ego.Scoeffs, 2); // s acc

    ego.dv = jerkEvalutate(ego.Dcoeffs, 1); // d velocity
    ego.da = jerkEvalutate(ego.Dcoeffs, 2); // d acc

    vector<Trajectory> traj = generateTraj(interpolate_waypoints_x, interpolate_waypoints_y, interpolate_waypoints_s);
    ego_next_x_vals = traj[0].x;
    ego_next_y_vals = traj[0].y;
    ego.Scoeffs = traj[0].Scoeffs;
    ego.Dcoeffs = traj[0].Dcoeffs;

#endif

}

void Vehicle::interpolate_waypoints(
    int ego_closest_point_idx, 
    vector<double>& interpolate_waypoints_x,
    vector<double>& interpolate_waypoints_y,
    vector<double>& interpolate_waypoints_s){

    vector<double> coarse_waypoints_x;
    vector<double> coarse_waypoints_y;
    vector<double> coarse_waypoints_s;

    for(int i = -NUM_WAYPOINTS_BEHIND; i < NUM_WAYPOINTS_AHEAD; i++){
        coarse_waypoints_x.push_back(map_waypoints_x[ego_closest_point_idx + i]);
        coarse_waypoints_y.push_back(map_waypoints_y[ego_closest_point_idx + i]);
        coarse_waypoints_s.push_back(map_waypoints_s[ego_closest_point_idx + i]);
    }

    double gap = 0.5; 
    int num_interpolate_points = (coarse_waypoints_s.back() - coarse_waypoints_s.front()) / gap;
    log << "interpolate waypoints s: (" << num_interpolate_points << ")" << endl;
    for(int i = 0; i < num_interpolate_points; i++){
        double s = coarse_waypoints_s.front() + i * gap;
        interpolate_waypoints_s.push_back(s);
        log << s << " ";
    }
    log << endl;
    interpolate_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, 
                                    gap, num_interpolate_points);
    interpolate_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, 
                                    gap, num_interpolate_points);

    log << "interpolate waypoints x: (" << num_interpolate_points << ")" << endl;
    for(auto i : interpolate_waypoints_x) log << i << " ";
    log << endl;
    log << "interpolate waypoints y: (" << num_interpolate_points << ")" << endl;
    for(auto i : interpolate_waypoints_y) log << i << " ";
    log << endl;
    
}

vector<Trajectory> Vehicle::generateTraj(vector<double>& map_x, 
                                vector<double>& map_y, 
                                vector<double>& map_s){
    
    vector<Trajectory> traj;

    for(int d = 6; d < 7; d += 4){
        Trajectory t;
        int closest_point_idx = ClosestWaypoint(ego.x, ego.y, map_x, map_y);
        int next_wappoint_idx = NextWaypoint(ego.x, ego.y, ego.yaw, map_x, map_y) + 20;
        
        double closest_x = map_x[closest_point_idx];
        double closest_y = map_y[closest_point_idx];
        double closest_s = map_s[closest_point_idx];

        double next_x = map_x[next_wappoint_idx];
        double next_y = map_y[next_wappoint_idx];
        double next_s = map_s[next_wappoint_idx];

        vector<double> start_pos_s {ego.s, ego.sv, ego.sa};
        vector<double> start_pos_d {ego.d, ego.dv, ego.da};
        vector<double> end_pos_s {next_s, mph2mps(49), 0};
        vector<double> end_pos_d {(double)d, 0, 0};
        log << "=== NEW Traj ===" << endl;
        log << "start s:" << ego.s << " ---> " << next_s << endl;
        log << "start d:" << ego.d << " ---> " << (double)d << endl;
        // calculate minimization jerk coefficients
        t.Scoeffs = jerkCoeffsSolve(start_pos_s, end_pos_s, 1);
        t.Dcoeffs = jerkCoeffsSolve(start_pos_d, end_pos_d, 1);
        log << "Scoeffs && Dcoeffs:" << endl;
        for(auto c : t.Scoeffs) log << c << " ";
        log << endl;
        for(auto c : t.Dcoeffs) log << c << " ";
        log << endl;

        vector<vector<double>> jerkMinTraj = jerkInterpolation(t.Scoeffs, t.Dcoeffs);
        vector<double> S = jerkMinTraj[0];
        vector<double> D = jerkMinTraj[1];

        log << "S points:(" << S.size() << ")" << endl;
        for(auto s : S) log << s << " ";
        log << endl << "D points:(" << D.size() << ")" << endl;
        for(auto d : D) log << d << " ";

        vector<double> X;
        vector<double> Y;

        for(int i = 0; i < S.size(); i++){
            double s = S[i];
            double d = D[i];
            vector<double> posXY = getXY(s, d, map_s, map_x, map_y);
            X.push_back(posXY[0]);
            Y.push_back(posXY[1]);
        }

        tk::spline smooth;
        smooth.set_points(X, Y);
        
        vector<double> next_x_trajectory;
        vector<double> next_y_trajectory;

        log << "New Trajectory: " << endl;
        double step = mph2mps(49) / 50;
        for(int i = 0; i < 50; i++){
            double x = ego.x + step * i;
            double y = smooth(x);
            t.x.push_back(x);
            t.y.push_back(y);
            log << "[" << x << "," << y << "] ";
        }

        t.s = S;
        t.d = D;
        traj.push_back(t);
        log << endl;
    }

    return traj;
}