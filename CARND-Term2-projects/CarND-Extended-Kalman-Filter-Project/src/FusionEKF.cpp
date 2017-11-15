#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;
    
    //measurement matrix
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    
    // noise for the process covariance
    noise_ax = 9;
    noise_ay = 9;
    
    //mtate matrix
    ekf_.x_ = VectorXd(4);
    // State covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    
    //transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<  1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::Init(const MeasurementPackage &measurement_pack) {

    if (is_initialized_) {
        return;
    }
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ << 1, 1, 1, 1;
    
    float px = 0, py = 0, vx = 0, vy = 0;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Convert radar from polar to cartesian coordinates and initialize state.
        float ro = measurement_pack.raw_measurements_(0);
        float phi = measurement_pack.raw_measurements_(1);
        float ro_dot = measurement_pack.raw_measurements_(2);
        px = ro * cos(phi);
        py = ro * sin(phi);
        vx = ro_dot * cos(phi);
        vy = ro_dot * sin(phi);
        
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state.
        px = measurement_pack.raw_measurements_(0);
        py = measurement_pack.raw_measurements_(1);
        
    }
    
    ekf_.x_ << px, py, vx, vy;
    
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;

}

float FusionEKF::ProcessTimestamp(const long timestamp) {
    float dt = (timestamp - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = timestamp;
    return dt;
}

/**
 * Add the elapsed time since last
 * measurement into the state transition matrix
 */
void FusionEKF::ProcessStateTransMatrix(const float dt) {
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
}

/**
 * Calculate the Qv and G matrix to get the
 * process covariance matrix `Q`
 */
void FusionEKF::ProcessCovarianceMatrix(const float dt) {
    // Noise covariance matrix computation
    MatrixXd Qv = MatrixXd(2, 2);
    Qv <<   noise_ax, 0,
            0, noise_ay;
    
    MatrixXd G = MatrixXd(4, 2);
    G <<    dt * dt / 2, 0,
            0, dt * dt / 2,
            dt, 0,
            0, dt;
    
    // calculate process covariance matrix
    ekf_.Q_ = G * Qv * G.transpose();
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

 
    /* Initialization */
    FusionEKF::Init(measurement_pack);
    
    /* Prediction */
    float dt = ProcessTimestamp(measurement_pack.timestamp_);
    ProcessStateTransMatrix(dt);
    ProcessCovarianceMatrix(dt);
    ekf_.Predict();
    
    /* Update */
    VectorXd z_pred;
    switch (measurement_pack.sensor_type_) {
        case MeasurementPackage::RADAR:
            try {
                ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
                ekf_.R_ = R_radar_;
                z_pred = ekf_.RadarUpdate();
            } catch(...) {
                // Jacobian error with a very small value -- ignore current update
                return;
            }
            break;
        case MeasurementPackage::LASER:
            ekf_.H_ = H_laser_;
            ekf_.R_ = R_laser_;
            z_pred = ekf_.LaserUpdate();
            break;
        default:
            break;
    }
    
    // update either RADAR or LASER measurement
    ekf_.Update(measurement_pack.raw_measurements_,
                z_pred);
}

