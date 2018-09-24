#include "FusionEKF.h"
#include <iostream>
#include <fstream>
#include "../tools/tools.h"
#include "../Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF(ofstream& log) : log_(log){
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225,   0,
              0,        0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09,   0,        0,
              0,      0.0009,   0,
              0,      0,        0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	H_laser_ << 1, 0, 0, 0,
		          0, 1, 0, 0; 
	Hj_ << 1, 1, 0, 0,
	       1, 1, 0, 0,
	       1, 1, 1, 1;
	ekf_.F_ = MatrixXd(4 ,4);
	ekf_.F_ <<  1, 0, 1, 0,
		          0, 1, 0, 1,
		          0, 0, 1, 0,
		          0, 0, 0, 1;
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ <<  1, 0, 0, 0,
		          0, 1, 0, 0,
		          0, 0, 1, 0,
		          0, 0, 0, 1;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    log_ << "Initialization" << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      log_ << "RADAR ro:" << ro << " phi:" << phi << " ro_dot:" << ro_dot << endl;
      ekf_.x_(0) = ro * cos(phi);
      ekf_.x_(1) = ro * sin(phi);
      ekf_.x_(2) = ro_dot * cos(phi);
      ekf_.x_(3) = ro_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      double x = measurement_pack.raw_measurements_(0);
      double y = measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = x;
      ekf_.x_(1) = y;
      log_ << "LASER x:" << x << " y:" << y << endl;
    }

    // done initializing, no need to predict or update
	  previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // We can use the timestamp values to compute 
  // the elapsed time between two consecutive observations as:
	float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

  // The noise is given
	float noise_ax = 9;
	float noise_ay = 9;

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
		   0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
		   dt_3/2 * noise_ax, 0, dt_2*noise_ax, 0,
		   0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;


  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);	
  }

}

VectorXd FusionEKF::getEstimate(){
  VectorXd estimate(4);

  double p_x = ekf_.x_(0);
  double p_y = ekf_.x_(1);
  double v1 = ekf_.x_(2);
  double v2 = ekf_.x_(3);

  estimate(0) = p_x;
  estimate(1) = p_y;
  estimate(2) = v1;
  estimate(3) = v2;

  return estimate;
}

vector<double> FusionEKF::getPos(){
  double p_x = ekf_.x_(0);
  double p_y = ekf_.x_(1);
  vector<double>pos = {p_x, p_y};
  return pos;
}