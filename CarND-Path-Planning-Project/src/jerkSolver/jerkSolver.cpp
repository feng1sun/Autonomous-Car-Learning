#include <vector>
#include <cassert>
#include <iostream>
#include "jerkSolver.h"
#include "../utils/utils.h"
#include "../utils/spline.h"
#include "../utils/Eigen-3.3/Eigen/Dense"
#include "../vehicle/vehicle.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> jerkCoeffsSolve(vector<double>& start, vector<double>& end, double T){
    assert(start.size() == 3);
    assert(end.size() == 3);

    double x0 = start[0]; 
    double vx0 = start[1];
    double ax0 = start[2];
    double x1 = end[0];
    double vx1 = end[1];
    double ax1 = end[2];

    // pre-computed exponentials of T:
    std::vector<double> T_exp{1.0, T, pow(T, 2.0), pow(T, 3.0),pow(T, 4.0), pow(T, 5.0)};
    
    // system matrix:
    Eigen::MatrixXd A(3, 3); 
    A <<   T_exp[3],    T_exp[4],    T_exp[5], 
         3*T_exp[2],  4*T_exp[3],  5*T_exp[4], 
         6*T_exp[1], 12*T_exp[2], 20*T_exp[3];
    
    // desired output:
    Eigen::VectorXd b(3);

    b <<  x1 - (x0 + vx0*T_exp[1] + 0.5*ax0*T_exp[2]),
         vx1 - (vx0 + ax0*T_exp[1]),
         ax1 - ax0;

    MatrixXd Ai = A.inverse();
	VectorXd c = Ai*b;

    return {x0, vx0, 0.5*ax0, c(0), c(1), c(2)};
}

vector<vector<double>> jerkInterpolation(const vector<double>& Scoeffs, const vector<double>& Dcoeffs){
    assert(Scoeffs.size() == 6);
    assert(Dcoeffs.size() == 6);
    vector<double> S;
    vector<double> D;

    for(int i = 0; i < 50; i ++){
        double T = 0.02 * i;
        double T2 = pow(T, 2.0);
        double T3 = pow(T, 3.0);
        double T4 = pow(T, 4.0);
        double T5 = pow(T, 5.0);
        double s = Scoeffs[0]
                + Scoeffs[1] * T
                + Scoeffs[2] * T2
                + Scoeffs[3] * T3
                + Scoeffs[4] * T4
                + Scoeffs[5] * T5;
        double d = Dcoeffs[0]
                + Dcoeffs[1] * T
                + Dcoeffs[2] * T2
                + Dcoeffs[3] * T3
                + Dcoeffs[4] * T4
                + Dcoeffs[5] * T5;

        S.push_back(s);
        D.push_back(d);
    }

    return vector<vector<double>> {S, D};
}

double jerkEvalutate(const vector<double>& coeffs, int order){
    double t = 0.02;
    double a0 = coeffs[0];
    double a1 = coeffs[1];
    double a2 = coeffs[2];
    double a3 = coeffs[3];
    double a4 = coeffs[4];
    double a5 = coeffs[5];
    switch(order){
        case 0: // s
            return a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
        case 1:// velocity
            return a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
        case 2:// acc
            return 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);
        default: cerr << "unexpected order!";
    }
}