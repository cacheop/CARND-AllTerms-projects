#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//////////////////
// constructor
//////////////////
    FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;
    
    //measurement matrix
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    // jacobian
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrices
    //laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
                0, 0.0225;
        
    //radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    
    // noise for the process covariance
    noise_ax = 9;
    noise_ay = 9;
    
    //state matrix
    ekf_.x_ = VectorXd(4);
        
    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    
    //transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<  1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
}

//////////////////
// destructor
//////////////////
FusionEKF::~FusionEKF() {}

//////////////////
// processing measurement
//////////////////
void FusionEKF::Init(const MeasurementPackage &measurement_pack) {

    // after 1st measurement
    if (is_initialized_) {
        return;
    }
    
    // 1st measurement
    cout << "EKF: " << endl;
    ekf_.x_ << 1, 1, 1, 1;
    
    // position and velocity
    float px = 0, py = 0, vx = 0, vy = 0;
    
   // radar data
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // convert from polar to cartesian coordinates
        float ro = measurement_pack.raw_measurements_(0);
        float phi = measurement_pack.raw_measurements_(1);
        float ro_dot = measurement_pack.raw_measurements_(2);
        
        // get measurement position and velocity
        px = ro * cos(phi);
        py = ro * sin(phi);
        vx = ro_dot * cos(phi);
        vy = ro_dot * sin(phi);
        
    }
    // laser data
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // get measurement position
        px = measurement_pack.raw_measurements_(0);
        py = measurement_pack.raw_measurements_(1);

    }
    
    // move measurement to object
    ekf_.x_ << px, py, vx, vy;
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    
    is_initialized_ = true;
    // update timestamp for next iteration
    previous_timestamp_ = measurement_pack.timestamp_;

}

//////////////////
// calculate delta T (time elapsed since last measurement)
//////////////////
float FusionEKF::ProcessTimestamp(const long timestamp) {
    float dt = (timestamp - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = timestamp;
    //std::cout << "dt" << dt << std::endl;
    
    //update elapsed time into state transition matrix
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    return dt;
}

//////////////////
// ProcessMeasurement
//////////////////
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

 
    // initialize
    FusionEKF::Init(measurement_pack);
    
    // predict
    float dt = ProcessTimestamp(measurement_pack.timestamp_);
    MatrixXd Qv = MatrixXd(2, 2);
    Qv <<   noise_ax, 0,
    0, noise_ay;
    
    MatrixXd G = MatrixXd(4, 2);
    G <<    dt * dt / 2, 0,
    0, dt * dt / 2,
    dt, 0,
    0, dt;
    
    // calculate the Qv and G matrices in order to get the process covariance matrix Q
    // calculate process covariance matrix
    ekf_.Q_ = G * Qv * G.transpose();
    ekf_.Predict();
    
    // preprocess before update
    VectorXd z_pred;
    switch (measurement_pack.sensor_type_) {
        case MeasurementPackage::RADAR:
            std::cout << "RADAR" << std::endl;
            try {
                ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
                ekf_.R_ = R_radar_;
                z_pred = ekf_.RadarUpdate();
            } catch(...) {
                return;
            }
            break;
        case MeasurementPackage::LASER:
            std::cout << "LASER" << std::endl;
            ekf_.H_ = H_laser_;
            ekf_.R_ = R_laser_;
            z_pred = ekf_.LaserUpdate();
            break;
        default:
            break;
    }
    
    // update - generic for RADAR or LASER
    ekf_.Update(measurement_pack.raw_measurements_,z_pred);
}

