#pragma once
#include <vector>

using namespace std;

class Trajectory{
public:
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> d;
    vector<double> Scoeffs;
    vector<double> Dcoeffs;
};