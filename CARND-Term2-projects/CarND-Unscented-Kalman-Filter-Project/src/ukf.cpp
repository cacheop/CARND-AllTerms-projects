#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;
    
    use_laser_ = true; // if false, laser measurements ignored (except during init)
    //use_laser_ = false;
    
    use_radar_ = true;  // if false, radar measurements ignored (except during init)
    //use_radar_ = false;

    n_x_ = 5;   // state dimension
    n_aug_ = 7; // augmented dimension
    n_z_radar_ = 3;   // measurement dimension, radar can measure r, phi, and r_dot
    n_z_laser_ = 2;   // measurement dimension px, py
    
    x_ = VectorXd(n_x_);     // initial state vector

    P_ = MatrixXd(5, 5); // initial covariance matrix
    P_.fill(0.);
    
    P_(0,0) = .1;
    P_(1,1) = .1;
    P_(2,2) = 1;
    P_(3,3) = 1;
    P_(4,4) = 1;
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    Xsig_pred_.fill(0.0);
    
    Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug_.fill(0.0);

    VectorXd z_pred = VectorXd(n_z_laser_);      // mean predicted measurement
    z_pred.fill(0.0);

    S_radar_ = MatrixXd(n_z_radar_, n_z_radar_);     // measurement covariance matrix S 3x3
    S_radar_.fill(0.0);

    S_laser_= MatrixXd(n_z_laser_, n_z_laser_);            // measurement covariance matrix S 2x2
    S_laser_.fill(0.0);

    R_radar_ = MatrixXd(3,3);                              // noise matrix: radar
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;
    
    R_laser_ = MatrixXd(2,2);                              // noise matrix: laser
    R_laser_ << std_laspx_* std_laspx_, 0,
                0, std_laspy_* std_laspy_;
    
    NIS_radar_ = 0;  //  NIS radar
    NIS_laser_ = 0;  //  NIS laser
    
    counter_ = 0;
    
    x_ << 1, 1, 1, 1, 1;        // first measurement
    
    lambda_ = 3 - n_aug_;           // spreading parameter

    // calculate weights
    double weight_0 = lambda_/(lambda_ + n_aug_);
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_.fill(0.0);
    weights_(0) = weight_0;
    for (int i=1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }
}

UKF::~UKF() {}
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    
    if (!is_initialized_) {
        cout << "Initializing UKF. Buckle your seatbelts!" << endl;

        double px = 0.;
        double py = 0.;
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_(0);
            if (rho < 0.001) rho = 0.001;

            double phi = meas_package.raw_measurements_(1);
            if (phi < 0.001) phi = 0.001;

            double rho_dot = meas_package.raw_measurements_(2);
            if (rho_dot < 0.001) rho_dot = 0.001;

            px = rho * cos(phi);
            if (px < 0.001) px = 0.001;

            py = rho * sin(phi);
            if (py < 0.001) py = 0.001;
            
            x_(0) = px;
            x_(1) = py;
            x_(2) = rho_dot;
            x_(3) = phi;
            
            //x_ << px, py, rho_dot, phi, 0;
 
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            px = meas_package.raw_measurements_(0);
            if (px < 0.001) px = 0.001;

            py = meas_package.raw_measurements_(1);
            if (py < 0.001) py = 0.001;
            x_ << px, py, 1, 1, 0.1;
            
    
        }
        
        is_initialized_ = true;
        previous_timestamp_ = meas_package.timestamp_;

        return;
    }
    
    // compute the time from the previous measurement in seconds.
    double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    
    Prediction(delta_t);
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        counter_++;
        UpdateRadar(meas_package);
    }
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        counter_++;
        UpdateLidar(meas_package);
    }
    
}

/////////////////////////////////////////////////////////////
//ring angle value to range [-PI, PI]
/////////////////////////////////////////////////////////////
double UKF::NormalizeAngle(double angle) {
    
    while (angle >  M_PI) {
        angle -= 2. * M_PI;
    }
    
    while (angle < -M_PI) {
        angle += 2. * M_PI;
    }

    return angle;
}


/////////////////////////////////////////////////////////////
// Generate sigma points
/////////////////////////////////////////////////////////////
void UKF::GenerateSigmaPoints() {
    lambda_ = 3 - n_x_;
    
    // calculate square root of P
    MatrixXd A = MatrixXd::Zero(n_aug_,n_aug_);
    A = P_.llt().matrixL();
    
    // set first column of sigma point matrix
    Xsig_pred_.col(0) = x_;
    
    //set remaining sigma points
    for (int i = 0; i < n_x_; i++) {
        Xsig_pred_.col(i + 1) = x_ + sqrt(lambda_+n_x_) * A.col(i);
        Xsig_pred_.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
    }
    
}

/////////////////////////////////////////////////////////////
// Augment the sigma points
/////////////////////////////////////////////////////////////
void UKF::AugmentedSigmaPoints() {
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.fill(0.0);
    
    //create augmented state covariance
    lambda_ = 3 - n_aug_;
    
    //create augmented mean state
    x_aug.head(5) = x_;

    
    //create augmented covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_ * std_a_;
    P_aug(6,6) = std_yawdd_ * std_yawdd_;
    
    //create square root matrix
    MatrixXd L;
    L.fill(0.0);
    L = P_aug.llt().matrixL();
    
    //create augmented sigma points
    Xsig_aug_.col(0)  = x_aug;
    
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
        Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

}

/////////////////////////////////////////////////////////////
// Predict sigma points
/////////////////////////////////////////////////////////////
void UKF::SigmaPointPrediction(double delta_t) {
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        //extract values for better readability
        double p_x = Xsig_aug_(0,i);
        double p_y = Xsig_aug_(1,i);
        double v = Xsig_aug_(2,i);
        double yaw = Xsig_aug_(3,i);
        double yawd = Xsig_aug_(4,i);
        double nu_a = Xsig_aug_(5,i);
        double nu_yawdd = Xsig_aug_(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v/yawd * (sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;
        
        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;

        yawd_p = yawd_p + nu_yawdd * delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i)= px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
        
    }
}

/////////////////////////////////////////////////////////////
// Predict mean and covariance matrix
/////////////////////////////////////////////////////////////
void UKF::PredictMeanAndCovariance() {

    // predicted state mean
    x_.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ += weights_(i) * Xsig_pred_.col(i);
    }
    
    // initialize covariance matrix for prediction
    P_.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        // normalize angle
        x_diff(3) = NormalizeAngle(x_diff(3));         // normalize angle

        P_ += weights_(i) * x_diff * x_diff.transpose() ;
        
    }

    //std::cout << "Predicted x: " << std::endl << x_ << std::endl;
    //std::cout << "Predicted P: " << std::endl << P_ << std::endl;
}

/////////////////////////////////////////////////////////////
// Predict stage
/////////////////////////////////////////////////////////////
void UKF::Prediction(double delta_t) {
    GenerateSigmaPoints();
    AugmentedSigmaPoints();
    SigmaPointPrediction(delta_t);
    PredictMeanAndCovariance();
}


/////////////////////////////////////////////////////////////
// Update laser
/////////////////////////////////////////////////////////////
void UKF::UpdateLidar(MeasurementPackage meas_package) {

    //set measurement dimension, lidar can measure px and py
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_laser_); //@@@ dimansion
    z_pred.fill(0.0);

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);
    Zsig.fill(0.0);

    //measurement covariance matrix S
    S_laser_.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd state_vec = Xsig_pred_.col(i);
        double px = state_vec(0);                       // transform sigma points into measurement space
        double py = state_vec(1);
        
        Zsig.col(i) << px,py;
        z_pred += weights_(i) * Zsig.col(i);            // calculate mean predicted measurement
    }
    
    //calculate measurement covariance matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S_laser_ += weights_(i) * z_diff * z_diff.transpose();
    }
    
    S_laser_ += R_laser_;                               // add R (noise) to S
    
    //create vector for incoming radar measurement
    VectorXd z = VectorXd(n_z_laser_);
    z.fill(0.0);
    
    double meas_px = meas_package.raw_measurements_(0);
    double meas_py = meas_package.raw_measurements_(1);

    z << meas_px, meas_py;
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);
    Tc.fill(0.0);
    Tc.fill(0.0);
    
    //calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = NormalizeAngle(x_diff(3));              // normalize angle
        VectorXd z_diff = Zsig.col(i) - z_pred;
        Tc += weights_(i) * x_diff * z_diff.transpose();
        
    }
    
    VectorXd z_diff = z - z_pred;                                       // residual
    NIS_laser_ = z_diff.transpose() * S_laser_.inverse() * z_diff;      //calculate NIS
    MatrixXd K = Tc * S_laser_.inverse();                               //calculate Kalman gain K;

    x_ += K * z_diff;                                                   //update state mean and covariance matrix
    P_ -= K * S_laser_ * K.transpose();
}

/////////////////////////////////////////////////////////////
// Update radar
/////////////////////////////////////////////////////////////
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
    Zsig.fill(0.0);
 
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //calculate measurement covariance matrix S
    VectorXd z_diff(n_z_radar_);
    z_diff.fill(0.0);


    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_radar_);
    z_pred.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred +=  weights_(i) * Zsig.col(i);
    }
    
    z_pred(1) = NormalizeAngle(z_pred(1));
    
    //calculate measurement covariance matrix S_radar
    S_radar_.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++ ) {
        z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = NormalizeAngle(z_diff(1));
        S_radar_ += weights_(i) * z_diff * z_diff.transpose();
    }
    
    S_radar_ += R_radar_;                                                // Add R (noise) to S
    
    // vector for incoming radar measurement
    VectorXd z = VectorXd(n_z_radar_);
    
    double meas_rho = meas_package.raw_measurements_(0);
    double meas_phi = meas_package.raw_measurements_(1);
    double meas_rhod = meas_package.raw_measurements_(2);
    
    z << meas_rho, meas_phi, meas_rhod;
    
    // matrix for cross correlation Tc

    MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
    Tc.fill(0.0);

    VectorXd x_diff;
    x_diff = VectorXd(n_aug_);
    x_diff.fill(0.0);

    // calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        
        x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = NormalizeAngle(x_diff(3));
        
        z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = NormalizeAngle(z_diff(1));

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    // calculate Kalman gain K;
    MatrixXd K_radar;
    K_radar = MatrixXd(n_x_, n_z_radar_);
    K_radar.fill(0.0);
    
    K_radar = Tc * S_radar_.inverse();

    // update state mean and covariance matrix
    z_diff = meas_package.raw_measurements_ - z_pred;
    z_diff(1) = NormalizeAngle(z_diff(1));
    
    // update state mean and covariance matrix
    x_ += K_radar * z_diff;
    P_ -= K_radar * S_radar_ * K_radar.transpose();
    
    // calculate NIS
    NIS_radar_ = z_diff.transpose() * S_radar_.inverse() * z_diff;
}


