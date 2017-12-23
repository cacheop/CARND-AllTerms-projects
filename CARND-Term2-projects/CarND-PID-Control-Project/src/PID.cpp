#include <math.h>
#include <iostream>

#include "pid.h"
using namespace std;


// constructor
PID::PID() {}

// destructor
PID::~PID() {}

// initialize PID object
void PID::Init(double K_p, double K_i, double K_d) {
    p_error = 0;
    i_error = 0;
    d_error = 0;
    
    Kp = K_p;
    Ki = K_i;
    Kd = K_d;
    
    iteration = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;

    iteration++;
}

double PID::TotalError() {
    return  -Kp * p_error - Ki * i_error - Kd * d_error;
}


void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


