#include "measurement_package.h"
#include "../Eigen/Dense"
#include <iostream>

using namespace std;

/**
 *  @param  istringstream
 *  @return ground truth VectorXd
 */
Eigen::VectorXd MeasurementPackage::read(istringstream& iss){
    long long timestamp;

    // reads first element from the current line
    string sensor_type;
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
        sensor_type_ = MeasurementPackage::LASER;
        raw_measurements_ = Eigen::VectorXd(2);
        float px;
        float py;
        iss >> px;
        iss >> py;
        raw_measurements_ << px, py;
        iss >> timestamp;
        timestamp_ = timestamp;
    } else if (sensor_type.compare("R") == 0) {

        sensor_type_ = MeasurementPackage::RADAR;
        raw_measurements_ = Eigen::VectorXd(3);
        float ro;
        float theta;
        float ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        raw_measurements_ << ro,theta, ro_dot;
        iss >> timestamp;
        timestamp_ = timestamp;
    }

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    Eigen::VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt; 
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;

    return gt_values;
}