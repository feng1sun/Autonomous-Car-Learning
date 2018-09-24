#ifndef PID_H
#define PID_H
#include <fstream>
#include <string>
#include <vector>
#include "../tools/json.hpp"

using json = nlohmann::json;
using namespace std;

class PID {
public:
  /*
  * Errors
  * error_[0] = p_error_;
  * error_[1] = i_error_;
  * error_[2] = d_error_;
  */
  vector<double> error_;
	

  /*
  * Coefficients
  * K_[0] = Kp
  * K_[1] = Ki
  * K_[2] = Kd
  */ 
  vector<double> K_;

  /*
  * Constructor
  * @param:
  *   path: json log save path.
  */
  PID(string path);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  json log_;
  ofstream save_;
  string path_;
  vector<double> dp_;
  double cte_prev_;
};

#endif /* PID_H */
