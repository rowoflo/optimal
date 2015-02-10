//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "system.h"


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
N_(state_dim),
M_(input_dim),
L_(output_dim)
{
    dynamics = [=] (StateType x, VectorXd u, TimeType t)
    {
        return StateType::Zero(N_);
    };

    controller = [=] (StateType x, TimeType t)
    {
        return InputType::Zero(M_);
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

Trajectory System::simulate(StateType init_state,
                            TimeType init_time,
                            TimeType duration,
                            System::Direction dir)
{
    // Initialize integration variables
    Stepper stepper;
    stepper.initialize(init_state, init_time, duration*init_step_factor_);
    pair< TimeType, TimeType > time_interval;

    auto dyn = [this] (const StateType x, StateType &dxdt, const TimeType t)
    {
        dxdt = dynamics(x, input(x,t), t);
    };

    TimeType t = init_time;
    TimeType T = duration;
    TimeType tf = t + T;
    unsigned int k = 1;

    StateType x = init_state;
    InputType u(M_);

    Trajectory traj(N_,M_);

    // Simulate
    u = input(x, t);
    traj.push_back(t, x, u);
    while (t<tf) {
        time_interval = stepper.do_step(dyn);
        t = time_interval.second;
        while (k*ts_<t && k*ts_<tf) {
            stepper.calc_state(k*ts_, x);
            u = input(x, t);
            traj.push_back(k*ts_, x, u);
            k++;
        }
    }
    stepper.calc_state(tf, x);
    u = input(x, t);
    traj.push_back(k*ts_, x, u);

    return traj;
}


//------------------------------------------------------------------------------
// Protected Member Functions
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Private Member Functions
//------------------------------------------------------------------------------


} // namespace optimal
