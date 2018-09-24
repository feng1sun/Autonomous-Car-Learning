#include <vector>
#include <cmath>
#include <iostream>

#include "mpc.h"
#include "../msg/message.h"
#include "../bezier/bezier.h"
using namespace std;

MPC::MPC(){
    cost_ = 0;
}

MPC::~MPC(){

}

/********************************
 * vector<double> ptsx = j[1]["ptsx"];
 * vector<double> ptsy = j[1]["ptsy"];
 * double px = j[1]["x"];
 * double py = j[1]["y"];
 * double psi = j[1]["psi"];
 * double v = j[1]["speed"];
 * double a = j[1]["throttle"];
 * double delta = j[1]["steering_angle"];
********************************/
void MPC::Model(message& state){
    vector<double> ptsx = state.ptsx;
    vector<double> ptsy = state.ptsy;

    bezier(ptsx, ptsy, 6, Bx_ref, By_ref);

    double cte = pow(state.px-Bx_ref.at(0), 2) + pow(state.py-By_ref.at(0), 2);
    double dv = pow(state.v - 3, 2);
    cost_ += cte;
    cost_ += dv;
    
}

vector<double> MPC::Optimizer(){ 
    
}
