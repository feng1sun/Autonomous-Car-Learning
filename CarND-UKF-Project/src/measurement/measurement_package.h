#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "../Eigen/Dense"
#include <iostream>

using namespace std;

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  Eigen::VectorXd read(istringstream& iss);
};

#endif /* MEASUREMENT_PACKAGE_H_ */
