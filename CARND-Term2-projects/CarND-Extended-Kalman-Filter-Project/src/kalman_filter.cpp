#include <math.h>
#include "kalman_filter.h"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

MatrixXd KalmanFilter::CalculateKalmanGain() {
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    return K;
}

VectorXd KalmanFilter::LaserUpdate() {
    // calculate predicted value
    // by only using the H matrix
    VectorXd z_pred = H_ * x_;
    return z_pred;
}

VectorXd KalmanFilter::RadarUpdate() {
    // extract the raw values from the state
    float px = x_(0), py = x_(1), vx = x_(2), vy = x_(3);

    // convert Cartesian back to Polar
    float rho = sqrt(px*px + py*py);
    
    //check division by zero
    if (rho < .00001) {
        rho = sqrt(.001 * .001 + .001 * .001);
    }
    
    float phi = atan2(py, px);
    float rho_dot = (px*vx + py*vy)/sqrt(px*px + py*py);

    // predicted state vector
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rho_dot;
    return z_pred;
}

