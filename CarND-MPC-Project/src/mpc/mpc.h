#pragma once 
#include <vector>
#include <iostream>

#include "../msg/message.h"

using namespace std;

class MPC{
    public:
        vector<double> Bx_ref;
        vector<double> By_ref;
        double cost_;

        MPC();
        ~MPC();
        void Model(message& state);
        void getRefTrajectory(vector<double> x, vector<double> y);
        vector<double> Optimizer();
    private:


};
