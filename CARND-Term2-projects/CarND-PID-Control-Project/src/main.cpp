#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "pid.h"
#include <math.h>

const bool running_tests = false;
const double target_speed = 40;

int count = 1501;
int max_count = 1500;
int test_case = 0;


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;
    
    PID speed_control_pid = PID();
    PID steering_pid;

    steering_pid.Init(0.13, 0., 3.0);
    speed_control_pid.Init(0.3, 0, 0.02);
    
    h.onMessage([&steering_pid, &speed_control_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        
        double speed_value = 0.0;

        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    
                    
                    if (running_tests) {
                        if (count > max_count) {
                            count = 0;
                            switch (test_case) {
                                case 0 :
                                    steering_pid.Init(0.13, 0., 3.0);
                                    speed_control_pid.Init(0.3, 0, 0.02);
                                    break;
                                case 1 :
                                    steering_pid.Init(0.15, 0, 6);
                                    speed_control_pid.Init(0.15, 0.0, .5);
                                    break;
                                case 2 :
                                    steering_pid.Init(0.15, 0, 5);
                                    speed_control_pid.Init(0.2, 0, .25);
                                    break;
                                case 3 :
                                    steering_pid.Init(0.15, 0, 4);
                                    speed_control_pid.Init(0.15, 0, 0.1);
                                    break;
                                case 4 :
                                    steering_pid.Init(0.15, 0, 3);
                                    speed_control_pid.Init(0.15, 0, 0.05);
                                    max_count = 9999999999;
                                    break;
                            }
                            steering_pid.Restart(ws);
                            test_case++;
                        }
                    }
                    
                    double steer_value = 0.0;
                    steering_pid.UpdateError(cte);
                    steer_value = steering_pid.TotalError();
                    if (steer_value >  1.0) steer_value = 1.0;
                    if (steer_value < -1.0) steer_value = -1.0;
                    
                    double speed_error = speed - target_speed;
                    speed_control_pid.UpdateError(speed_error);
                    speed_value = speed_control_pid.TotalError();
                    
                    //std::cout << "[Test " << test_case << "/" << count << "]" << "  Steer params: " << steering_pid.Kp << " " << steering_pid.Ki << " " << steering_pid.Kd << "  Speed params: " << speed_control_pid.Kp << " " << speed_control_pid.Ki << " " << speed_control_pid.Kd << std::endl;
                    
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    //msgJson["throttle"] = 0.3;
                    msgJson["throttle"] = speed_value;
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                    count++;
                    
                }
            }
            else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}

