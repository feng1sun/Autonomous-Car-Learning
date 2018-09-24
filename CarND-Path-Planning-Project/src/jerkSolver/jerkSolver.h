#pragma once
#include <vector>
#include "../utils/spline.h"
#include "../vehicle/vehicle.h"

using namespace std;

vector<double> jerkCoeffsSolve(vector<double>& start, vector<double>& end, double T);

vector<vector<double>> jerkInterpolation(const vector<double>& Scoeffs, const vector<double>& Dcoeffs);

double jerkEvalutate(const vector<double>& coeffs, int order);