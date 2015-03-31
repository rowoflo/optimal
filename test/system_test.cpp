/**
 * @brief Test program for System class.
 *
 * @note
 *
 * @author Rowland O'Flaherty <rowlandoflaherty.com>
 *
 * @date 2015 FEB 06
 *
 * @copyright Copyright (C) 2015, see LICENSE file
 */

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "system.h"
#include "trajectory.h"
#include <Eigen/Dense>
#include <iostream>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace optimal;
using namespace Eigen;

//--------------------------------------------------------------------------
// Enums, Strcuts
//--------------------------------------------------------------------------
enum ControllerType {
    NONE = 0,
    CONSTANT = 1,
    PID = 2,
    LQR = 3
};

struct ControllerParams {
    double constant;
    vector<double> pid;
    vector<double> lqr;
};

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // System definition
    bool WIND = false;        ///< Wind flag
    double MASS = 1;          ///< Mass [kg]
    double GRAVITY = 10;      ///< Gravity [m/s^2]
    double DISTURBANCE = -2;  ///< Disturbance (wind) [N]
    double HEIGHT = 5;        ///< Desired height [m]

    TimeType t0 = 0.0;        ///< Initial time [s]
    TimeType tf = 8.0;        ///< Final time [s]
    TimeType ts = 0.01;        ///< Time step [s]

    const unsigned int n = 3; ///< State dimension
    StateType x0(n);          ///< Initial state vector
    x0 << 0, 0, 0;

    StateType x_bar(n);       ///< State operating point
    x_bar << HEIGHT, 0, 0;

    const unsigned int m = 1; ///< Input dimension
    InputType u_max(m);       ///< Maximum input
    u_max << 15.0;
    InputType u_min(m);       ///< Minimum input
    u_min << 0.0;
    InputType u_bar(m);       ///< Input operating point
    u_bar << MASS*GRAVITY;

    MatrixXd A(n,n);          ///< State transition matrix (n x n)
    A << 0, 1, 0,
         0, 0, 0,
         1, 0, 0;

    Vector3d B(n,m);          ///< Input matrix (n x m)
    B << 0, 1/MASS, 0;

    VectorXd G(n);            ///< Disturbance vector
    G << 0, (WIND ? DISTURBANCE/MASS : 0), 0;

    VectorXd Q(n);            ///< State cost matrix diagonal
    Q.setOnes();

    VectorXd R(m);            ///< Input cost matrix diagonal
    R.setOnes();

    VectorXd S(n);            ///< Final state cost matrix diagonal
    S << 1000, 1000, 1000;

    System sys(ts, n, m);     ///< System

    // System dynamics
    sys.dynamics = [&] (StateType x_, InputType u_, TimeType t_)
    {
        return A*(x_ - x_bar) + B*(u_ - u_bar) + G;
    };

    // Controller
    ControllerType controller_type = PID;

    ControllerParams params;
    params.constant = 12.0;
    params.pid = {10, .5, 5};

    switch (controller_type) {
        case NONE:
            params.constant = 0.0;

        case CONSTANT:
            sys.controller = [=] (StateType x_, TimeType t_) ///< System controller
            {
                InputType u_(m);
                u_ << params.constant;
                return u_;
            };
            break;

        case PID:
            sys.controller = [=] (StateType x_, TimeType t_) ///< System controller
            {
                double p_ = params.pid[0];
                double i_ = params.pid[1];
                double d_ = params.pid[2];
                Matrix<double, m, n> K(p_, d_, i_);
                InputType u_(m);
                u_ = -K*(x_ - x_bar) + u_bar;
                return u_;
            };
            break;

        default:
            cout << "Unknown controller type" << endl;
            return 1;
    }

    // Input constraint
    sys.input_constraint = [=] (InputType u_) ///< Input constraint
    {
        u_ = u_.array().min(u_max.array());
        u_ = u_.array().max(u_min.array());
        return u_;
    };

    // Cost
    sys.state_cost = [&] (StateType x_) ///< State cost
    {
        return (x_-x_bar).transpose() * Q.asDiagonal() * (x_-x_bar);
    };

    sys.input_cost = [&] (InputType u_) ///< Input cost
    {
        return u_.transpose() * R.asDiagonal() * u_;
    };

    sys.final_state_cost = [&] (StateType xf_) ///< Final state cost
    {
        return (xf_-x_bar).transpose() * S.asDiagonal() * (xf_-x_bar);
    };

    // Simulate
    auto tstart = chrono::high_resolution_clock::now();
    Trajectory traj = sys.simulate(x0, t0, tf-t0);
    auto elapsed = chrono::high_resolution_clock::now() - tstart;
    long long microseconds = chrono::duration_cast<chrono::microseconds>(elapsed).count();
    StateType xf = traj.state(traj.size()-1);
    cout << traj << endl;
    cout << xf.transpose() << endl;
    cout << "Run time: " << (double)microseconds/1e6 << endl;

    // StateType xf(n);
    // xf << 5.18125, -0.00929819, -3.53298;
    // Trajectory traj = sys.simulate(xf, tf, tf-t0, optimal::System::BACKWARD);

    // tstart = chrono::high_resolution_clock::now();
    // traj = sys.simulate(xf, tf, tf-t0, optimal::System::BACKWARD);
    // elapsed = chrono::high_resolution_clock::now() - tstart;
    // microseconds = chrono::duration_cast<chrono::microseconds>(elapsed).count();
    // cout << "Run time: " << (double)microseconds/1e6 << endl;


    // x0 = traj.state(traj.size()-1);
    // // cout << traj << endl;
    // cout << x0.transpose() << endl;



}
