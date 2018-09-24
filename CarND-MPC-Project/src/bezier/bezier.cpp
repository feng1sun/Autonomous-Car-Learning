#include <vector>
#include <cassert>
#include <cmath>

#include "./bezier.h"

using namespace std;

int BinormialCoeffs(int n, int k){
    int a = 1;
    int b = 1;
    int c = 1;
    for(int i = n; i > 0; i--){
        a *= i;
    }
    for(int i = k; i > 0; i--){
        b *= i;
    }
    for(int i = n - k; i > 0; i--){
        c *= i;
    }

    return a / (b * c);
}

void bezier(vector<double>& X, vector<double>& Y, int n,
             vector<double>& Bx, vector<double>& By)
{
    assert(X.size() == n);
    assert(Y.size() == n);
    Bx.clear();
    By.clear();
    n = n - 1;
    double t = 0;
    for(int i = 0; i < 100; i++){
        t += 0.01;
        double x = 0;
        double y = 0;
        for(int k = 0; k <= n; k++){
            x += BinormialCoeffs(n, k)*pow((1-t), n-k)*pow(t, k)*X[k];
            y += BinormialCoeffs(n, k)*pow((1-t), n-k)*pow(t, k)*Y[k];
        }
        Bx.push_back(x);
        By.push_back(y);
    }
}


