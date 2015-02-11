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
        // Stepper always integrates from 0 to allow for forward and backward
        // integration. The time is corrected in the loop and function call.
    stepper.initialize(init_state, 0, duration*init_step_factor_);
    pair< TimeType, TimeType > time_interval;

    auto dyn = [this, dir, t0] (const StateType x,
                                StateType &dxdt,
                                const TimeType t)
    {
        dxdt = dir*dynamics(x, input(x, dir*t+t0), dir*t+t0);
    };

    TimeType t = t0;
    TimeType tf = t0 + dir*duration;
    TimeType t2;
    int k = 1;

    StateType x = init_state;
    StateType xf;
    InputType u(m_);
    CostType L = 0;
    CostType cum_L = L;
    CostType final_L = final_cost(x, t);
    CostType J = cum_L + final_L;

    Trajectory traj(n_,m_);

    // Simulate
    u = input(x, t);
    traj.push_back(t, x, u, J);
    t = dir*k*ts_ + t0;
    while (dir*t < dir*tf) { // Step forward some interval until final time
        time_interval = stepper.do_step(dyn);
        t2 = t0 + dir*time_interval.second;
        // For all system time steps in interval record time, state, and input
        while (dir*t<dir*t2 && dir*t<dir*tf) {
            stepper.calc_state(dir*(t-t0), x);
            u = input(x, t);
            L = instantaneous_cost(x, u, t);
            cum_L += L;
            final_L = final_cost(x, t);
            J = cum_L + final_L;

            traj.push_back(t, x, u, J);
            k++;
            t = dir*k*ts_ + t0;
        }
    }
    stepper.calc_state(dir*(tf-t0), xf);
    u = input(xf, tf);
    final_L = final_cost(xf, tf);
    J = cum_L + final_L;
    traj.push_back(tf, xf, u, J);

    return traj;
}


//------------------------------------------------------------------------------
// Protected Member Functions
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Private Member Functions
//------------------------------------------------------------------------------


} // namespace optimal
