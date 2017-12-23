#ifndef PID_H
#define PID_H

#include <string>
#include <uWS/uWS.h>

class PID {
public:
    int iteration;
    const double tolerance = 5e-3;
    
    // Errors
    double p_error;
    double i_error;
    double d_error;

    // Coefficients
    double Kp;
    double Ki;
    double Kd;

    // Constructor
    PID();

    // Destructor.
    virtual ~PID();

    // Initialize PID.
    void Init(double Kp, double Ki, double Kd);

    // Update the PID error variables given cross track error.
    void UpdateError(double cte);

    // Calculate the total PID error.
    double TotalError();
    double TotalErrorTwiddler(double p, double i, double d);

    
    void TwiddleOld();
    void Twiddle();
    
    // restart simulation
    void Restart(uWS::WebSocket<uWS::SERVER> ws);

};

#endif /* PID_H */

