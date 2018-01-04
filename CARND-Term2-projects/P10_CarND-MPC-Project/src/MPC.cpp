#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;



// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 70;

// Set the timestep length and duration
size_t N = 12;
double dt = 0.06;

// Indexes for each data
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    
    FG_eval(Eigen::VectorXd coeffs) {
        this->coeffs = coeffs;
    }
    
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    
    ///////////////////////////////////////////////////
    // MPC
    // `fg` a vector of the cost constraints,
    // `vars` is a vector of variable values (state & actuators)
    ///////////////////////////////////////////////////
    void operator()(ADvector& fg, const ADvector& vars) {
    
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;
        
        // weights used to refine cost value
        const double cte_weight = 5000;
        const double epsi_weight = 5000;
        const double v_weight = 10;
        const double actuator_cost_weight = 5;
        const double steer_rate_cost_weight = 200000;
        const double accel__rate_cost_weight = 10;
        
        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            fg[0] += cte_weight * CppAD::pow(vars[cte_start + t] - ref_cte , 2);
            fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
            fg[0] += v_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
        }
        
        // Minimize change-rate.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += actuator_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += actuator_cost_weight * CppAD::pow(vars[a_start + t], 2);
        }
        
        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += steer_rate_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += accel__rate_cost_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }
        
        //
        // Setup Constraints
        //
        
        // Initial constraints
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];
        
        
        // Remaining constraints
        for (int t = 1; t < N; t++) {
            
            AD<double> x0 = vars[x_start + t - 1];          // The state at time t
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> epsi0 = vars[epsi_start + t - 1];
            
            AD<double> x1 = vars[x_start + t];             // The state at time t+1
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];
            
            // Only consider the actuation at time t
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];
            
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
            AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);
            

            // Using lesson equations for the model:
            // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
            // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
            // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
            // v_[t] = v[t-1] + a[t-1] * dt
            // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
            // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    
    // move state values into readable vars
    double x    = state[0];
    double y    = state[1];
    double psi  = state[2];
    double v    = state[3];
    double cte  = state[4];
    double epsi = state[5];
    
    // Set the number of model variables (includes both states and inputs): 4 * 10 + 2 * 9
    size_t n_vars = N * 6 + (N - 1) * 2;
    
    // Set the number of constraints
    size_t n_constraints = N * 6;

    // Initial value of the independent variables. SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0.0;
    }
    
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set lower and upper limits for variables.

    // Set non-actuators upper and lowerlimits to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    double max_degree   = 25;
    double max_radians  = max_degree * M_PI / 180;
    for (size_t i = delta_start; i < a_start; ++i) {
        vars_lowerbound[i] = - max_radians;
        vars_upperbound[i] = + max_radians;
    }

    
    // Acceleration/decceleration upper and lower limits.
    double max_acceleration  = 1.0;
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -max_acceleration;
        vars_upperbound[i] = max_acceleration;
    }
    
    // Lower and upper limits for the constraints. Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    
    // Force the solver to start from current state in optimization space
    constraints_lowerbound[x_start] = constraints_upperbound[x_start] = x;
    constraints_lowerbound[y_start] = constraints_upperbound[y_start] = y;
    constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = psi;
    constraints_lowerbound[v_start] = constraints_upperbound[v_start] = v;
    constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi;
    
    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true          forward\n";
    options += "Sparse  true          reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time            0.5\n";
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
                                          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
                                          constraints_upperbound, fg_eval, solution);
    
    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    
    // Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    vector<double> result;
    
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);
    
    for (int i = 0; i < N-1; i++)
    {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }
    
    return result;
}

