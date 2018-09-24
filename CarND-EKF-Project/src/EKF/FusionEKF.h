#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <vector>
#include <string>
#include <fstream>
#include "../Eigen/Dense"
#include "../tools/tools.h"
#include "../kalman/kalman_filter.h"
#include "../measurement/measurement_package.h"

class FusionEKF {
public:
  // Logger
  ofstream& log_;

  /**
  * Constructor.
  */
  FusionEKF(ofstream& log);

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

  /**
   * Get the current estimated x and y position 
   */
  VectorXd getEstimate();

  /**
   * Get Position {x, y}
   */
  vector<double> getPos();

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
