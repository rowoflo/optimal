//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "system.h"

#include <iostream>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace optimal
{

//------------------------------------------------------------------------------
// Friends
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Friend Overloaded Operators
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Nested Classes
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Lifecycle
//------------------------------------------------------------------------------
// Constructors
System::System(TimeType time_step,
               unsigned int state_dim,
               unsigned int input_dim,
               unsigned int output_dim)
:
ts_(time_step),
n_(state_dim),
m_(input_dim),
l_(output_dim)
{
    dynamics = [=] (StateType x, VectorXd u, TimeType t)
    {
        return StateType::Zero(n_);
    };

    controller = [=] (StateType x, TimeType t)
    {
        return InputType::Zero(m_);
    };

    time_cost = [=] (TimeType t)
    {
        return 0.0;
    };

    state_cost = [=] (StateType x)
    {
        return 0.0;
    };

    input_cost = [=] (InputType u)
    {
        return 0.0;
    };

    final_time_cost = [=] (TimeType tf)
    {
        return 0.0;
    };

    final_state_cost = [=] (StateType xf)
    {
        return 0.0;
    };

    state_constraint = [=] (StateType x)
    {
        return x;
    };

    input_constraint = [=] (InputType u)
    {
        return u;
    };
}

// Destructor


// Copy constructor



// Copy helper function



//------------------------------------------------------------------------------
// Overloaded Operators
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Getters and Setters
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Public Member Functions
//------------------------------------------------------------------------------
InputType System::input(StateType x, TimeType t)
{
    return input_constraint(controller(x, t));
}

CostType System::instantaneous_cost(StateType x, InputType u, TimeType t)
{
    return (state_cost(x) + input_cost(u) + time_cost(t))*ts_;
}

CostType System::final_cost(StateType xf, TimeType tf)
{
    return final_state_cost(xf) + final_time_cost(tf);
}

Trajectory System::simulate(StateType init_state,
                            TimeType init_time,
                            TimeType duration,
                            System::Direction dir)
{
    TimeType t0 = init_time;

    // Initialize integration variables
    Stepper stepper;
    auto dyn = [this, dir, t0] (const StateType x,
                                StateType &dxdt,
                                const TimeType t)
    {
        // cout << "t: " << t << ", u: " << input_interpolation(input(x, dir*t+t0)) << endl;
        dxdt = dir*dynamics(x, input_interpolation(input(x, dir*t+t0)), dir*t+t0);
    };

    TimeType t = t0;
    TimeType tf = t0 + dir*duration;
    TimeType t_stepper = 0;
    int k = 1;

    StateType x = init_state;
    StateType xf;
    CostType L = 0;
    CostType cum_L = L;
    CostType final_L = final_cost(x, t);
    CostType J = cum_L + final_L;

    Trajectory traj(n_,m_);

    // Simulate
    u_ = input(x, t);
    traj.reserve(floor(duration/ts_)+1);
    traj.push_back(t, x, u_, J);
    t = t0;
    StateType dxdt;
    while (dir*t < dir*tf - 1e-10) { // Step forward some interval until final time
        // t_stepper = dir*(t - t0);
        // stepper.do_step(dyn, x, t_stepper, ts_);
        t += dir*ts_;
        u_ = input(x, dir*t);
        dxdt = dir*dynamics(x, u_, t);
        x = x + dxdt * ts_;

        // u_ = input(x, t);
        L = instantaneous_cost(x, u_, t);
        cum_L += L;
        final_L = final_cost(x, t);
        J = cum_L + final_L;
        traj.push_back(t, x, u_, J);
    }
    return traj;
}


//------------------------------------------------------------------------------
// Protected Member Functions
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Private Member Functions
//------------------------------------------------------------------------------
InputType System::input_interpolation(InputType u)
{
    switch (input_interp_method) {
        case CONT:
            return u;
        case ZOH:
            return u_;
    }
}


} // namespace optimal
