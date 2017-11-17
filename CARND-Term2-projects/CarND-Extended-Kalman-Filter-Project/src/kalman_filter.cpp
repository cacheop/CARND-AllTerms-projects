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
    float phi = atan2(py, px);
    float rho_dot;
    
    //check division by zero
    if (fabs(rho) < .00001) {
        rho_dot = 0;
        std::cout << "whoa!" << std::endl;

    }
    else {
        rho_dot = (px*vx + py*vy)/rho;
    }

    // predicted state vector
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rho_dot;
    return z_pred;
}

void KalmanFilter::Update(const VectorXd &z, const VectorXd &z_pred) {
    
    // calculate Kalman Gain    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    // predicted error
    VectorXd y = z - z_pred;
    
    y[1] = atan2(sin(y[1]), cos(y[1]));
    
    // new state matrix using Kalman gain
    x_ = x_ + (K * y);
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    
    // state covariance matrix
    P_ = (I - K * H_) * P_;
}
