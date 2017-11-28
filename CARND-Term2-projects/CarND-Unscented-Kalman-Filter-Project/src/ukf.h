#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix & augmented
    MatrixXd Xsig_pred_;
    MatrixXd Xsig_aug_;

    ///* time when the state is true, in us
    long long time_us_;
    
    // previous timestamp
    long long previous_timestamp_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_ ;

    VectorXd weights_;      // weights of sigma points
    int n_x_;               // state dimensions
    
    int n_z_radar_;   // measurement dimensions
    int n_z_laser_;
    
    MatrixXd R_radar_;      // noise matrices
    MatrixXd R_laser_;
    
    MatrixXd S_radar_;     // measurement covariance matrix: radar
    MatrixXd S_laser_;   // measurement covariance matrix: laser
    
    int n_aug_;             // augmented state dimension
    double lambda_;         // sigma point sprexding parameter
        
    double NIS_radar_;      // NIS for radar
    double NIS_laser_;      // NIS for laser

    int counter_;

    //---------------------------------
    // Class methods
    //---------------------------------
    
    UKF();

    virtual ~UKF();

    double NormalizeAngle(double x);

    void ProcessMeasurement(MeasurementPackage meas_package);

    void Prediction(double delta_t);
    void Prediction2(double delta_t);


    void GenerateSigmaPoints();
    void AugmentedSigmaPoints();
    void SigmaPointPrediction(double delta_t);
    void PredictMeanAndCovariance();
 
    void UpdateLidar(MeasurementPackage meas_package);
    void UpdateRadar(MeasurementPackage meas_package);
};
#endif /* UKF_H */
