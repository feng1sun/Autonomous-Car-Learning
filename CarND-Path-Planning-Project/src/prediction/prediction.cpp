#include "../vehicle/vehicle.h"
#include "prediction.h"
#include <fstream>
#include <queue>

using namespace std;

vector<double> Predictor::predict(vector<vector<double>>& sensor_fusion){
    vector<obstacle> obstacleList;
    for(auto sf : sensor_fusion){
        obstacle car;
        car.id = sf[0];
        car.x = sf[1];
        car.y = sf[2];
        car.vx = sf[3];
        car.vy = sf[4];
        car.s = sf[5];
        car.d = sf[6];
        // check if car in the same side.
        if(car.d < 0) continue;
        obstacleList.push_back(car);
    }
}

double Predictor::costCollision(vector<obstacle> obstacleList){
    // 3 lane --> 3 trajectory
    log << endl;
    for(auto traj : ego_trajectory){
        for(auto car : obstacleList){
            log << "car id: " << car.id;
        }
    }
    log << endl;
}