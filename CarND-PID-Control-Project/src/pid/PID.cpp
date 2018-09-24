#include "PID.h"
#include <limits>
#include <iostream>
#include <string>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(string path) {
	error_.push_back(0);
	error_.push_back(0);
	error_.push_back(0);
	dp_.push_back(0.001);
	dp_.push_back(0.001);
	dp_.push_back(0.001);
	K_.push_back(0.1);
	K_.push_back(0.1);
	K_.push_back(0.01);
	cte_prev_ = 0;
	path_ = path;
}

PID::~PID() {
	save_.open(path_, ios_base::trunc);
	if(!save_.is_open()){
		throw "Json Save Not Open!";
	}
	save_ << log_ << endl;
	save_.close();
	cout << "Save log in: " << path_ << endl;
}

void PID::Init(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0) {
	K_[0] = Kp;
	K_[1] = Ki;
	K_[2] = Kd;
}

void PID::UpdateError(double cte) {
	double p_error = error_[0];
	double i_error = error_[1];
	double d_error = error_[2];

	d_error = cte - cte_prev_;
	// i_error = 0.2 * cte + 0.8 * i_error;
    i_error = 0.1 * cte + 0.9 * i_error;
	p_error = cte;
	cte_prev_ = cte;

	error_[0] = p_error;
	error_[1] = i_error;
	error_[2] = d_error;
	log_["cte"].push_back(cte);
	log_["d_error"].push_back(d_error);
	log_["i_error"].push_back(i_error);
	log_["p_error"].push_back(p_error);
}

double PID::TotalError() {
	double u = - (K_[0] * error_[0] + K_[1] * error_[1] + K_[2] * error_[2]); 

	log_["control"].push_back(u);
	log_["dp0"].push_back(dp_[0]);
	log_["dp1"].push_back(dp_[1]);
	log_["dp2"].push_back(dp_[2]);
	log_["Kp"].push_back(K_[0]);
	log_["Ki"].push_back(K_[1]);
	log_["Kd"].push_back(K_[2]);
	return u;
}

