#pragma once
#include <vector>
#include "../vehicle/vehicle.h"

using namespace std;

class Predictor : Vehicle {
public:

    typedef struct {
        int id;
        double x;
        double y;
        double s;
        double d;
        double vx;
        double vy;
    } obstacle;

    vector<double> predict(vector<vector<double>>& sensor_fusion);
    double costCollision(vector<obstacle> obstacleList);

    vector<vector<double>>& ego_trajectory;
};

