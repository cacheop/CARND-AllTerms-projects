#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:

  FusionEKF();
  virtual ~FusionEKF();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  KalmanFilter ekf_;

private:
    bool is_initialized_;
    long  previous_timestamp_;
    float noise_ax, noise_ay;

    
    Tools tools; // used to compute Jacobian and RMSE
    MatrixXd R_radar_;
    MatrixXd R_laser_;
    MatrixXd H_laser_;
    MatrixXd Hj_;

    void Init(const MeasurementPackage &measurement_pack);
    void ProcessStateTransMatrix(const float dt);
    void ProcessCovarianceMatrix(const float dt);
    float ProcessTimestamp(const long timestamp);

};

#endif /* FusionEKF_H_ */
