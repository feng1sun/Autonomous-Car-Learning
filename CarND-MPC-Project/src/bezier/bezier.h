#pragma once
#include <vector>

using namespace std;

int BinormialCoeffs(int n, int k);

void bezier(vector<double>& X, vector<double>& Y, int n,
             vector<double>& Bx, vector<double>& By);
