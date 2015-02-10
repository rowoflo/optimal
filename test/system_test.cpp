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
    TimeType ts = 0.1;        ///< Time step [s]

    const unsigned int N = 3; ///< State dimension
    StateType x0(N);          ///< Initial state vector
    x0 << 0, 0, 0;

    StateType x_bar(N);       ///< State operating point
    x_bar << HEIGHT, 0, 0;

    const unsigned int M = 1; ///< Input dimension
    InputType u_max(M);       ///< Maximum input
    u_max << 15.0;
    InputType u_min(M);       ///< Minimum input
    u_min << 0.0;
    InputType u_bar(M);       ///< Input operating point
    u_bar << MASS*GRAVITY;

    MatrixXd A(N,N);          ///< State transition matrix (N x N)
    A << 0, 1, 0,
         0, 0, 0,
         1, 0, 0;

    Vector3d B(N,M);          ///< Input matrix (N x M)
    B << 0, 1/MASS, 0;

    VectorXd G(N);            ///< Disturbance vector
    G << 0, (WIND ? DISTURBANCE/MASS : 0), 0;

    System sys(ts, N, M);     ///< System

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
                InputType u_(M);
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
                Matrix<double, M, N> K(p_, d_, i_);
                InputType u_(M);
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

    // Simulate
    Trajectory traj = sys.simulate(x0, t0, tf);

    // Print results
    cout << traj << endl;

}
