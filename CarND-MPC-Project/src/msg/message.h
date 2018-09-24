#pragma once 
#include <vector>
#include <iostream>

using namespace std;

class message{
    public:
        vector<double> ptsx;
        vector<double> ptsy;
        double px;
        double py;
        double psi;
        double v;
        double a;
				double delta;

        friend ostream& operator<<(ostream& os, const message& self);
        message();
        ~message();
 
    private:
};
