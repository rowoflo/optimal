/**
 * @file system.h
 *
 * @brief System class header file.
 *
 * @author <a href="http://rowlandoflaherty.com">Rowland O'Flaherty</a>
 *
 * @date 2015 FEB 06
 *
 * @copyright Copyright (C) 2015, see LICENSE file
 */

#ifndef OPTIMAL_SYSTEM_H_
#define OPTIMAL_SYSTEM_H_


//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "trajectory.h"

#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

namespace optimal
{

//------------------------------------------------------------------------------
// Typedefs
//------------------------------------------------------------------------------
// typedef dense_output_runge_kutta
//     <controlled_runge_kutta< runge_kutta_dopri5< StateType > > > Stepper;

typedef runge_kutta4< StateType > Stepper;


/**
* @class System
*
* @brief A class for dynamical systems.
*
* TODO: Detailed description
*/
class System
{
    //--------------------------------------------------------------------------
    // Friends
    //--------------------------------------------------------------------------



    //--------------------------------------------------------------------------
    // Friend Overloaded Operators
    //--------------------------------------------------------------------------



public:
    //--------------------------------------------------------------------------
    // Public Constants, Enums, and Types
    //--------------------------------------------------------------------------
    enum Direction {
        FORWARD = 1,
        BACKWARD = -1
    };

    enum InputInterpMethod {
        CONT = 0,
        ZOH = 1
    };


    //--------------------------------------------------------------------------
    // Public Nested Classes
    //--------------------------------------------------------------------------



    //--------------------------------------------------------------------------
    // Lifecycle
    //--------------------------------------------------------------------------
    // Constructors
    /**
     * @brief System constructor
     * @details Default constructor with ability to assign state, input, and
     * output dimensions.
     *
     * @param time_step Time step
     * @param state_dim State dimension
     * @param input_dim Input dimension
     * @param output_dim Output dimension
     */
    System(TimeType time_step=0.1,
           unsigned int state_dim=1,
           unsigned int input_dim=0,
           unsigned int output_dim=0);


    // Destructors


    // Copy constructor


    //--------------------------------------------------------------------------
    // Overloaded Operators
    //--------------------------------------------------------------------------



    //--------------------------------------------------------------------------
    // Getters and Setters
    //--------------------------------------------------------------------------
    void time(TimeType t) { t_ = t; };
    TimeType time() const { return t_; };
    void time_step(TimeType ts) { ts_ = ts; };
    TimeType time_step() const { return ts_; };
    void state_dim(unsigned int n) { n_ = n; };
    unsigned int state_dim() const { return n_; };
    void input_dim(unsigned int m) { m_ = m; };
    unsigned int input_dim() const { return m_; };
    void output_dim(unsigned int l) { l_ = l; };
    unsigned int ouput_dim() const { return l_; };


    //--------------------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------------------
    /**
     * @brief Outputs input from state and time.
     * @details Outputs system input based on state and time using the current
     *     controller and input constraints functions defined by the user.
     *
     * @param x State vector (n x 1)
     * @param t Time value
     *
     * @return Input vector (m x 1)
     */
    InputType input(StateType x, TimeType t);

    /**
     * @brief Outputs instantaneous cost from state, input, and time.
     * @details Outputs instantaneous cost based on the state cost, input cost,
     *     and time cost functions defined by the user.
     *
     * @param x State vector (n x 1)
     * @param u Input vector (m x 1)
     * @param t Time value
     * @return Instantaneous cost
     */
    CostType instantaneous_cost(StateType x, InputType u, TimeType t);

    /**
     * @brief Outputs final cost from state and time.
     * @details Outputs final cost based state final cost and time final cost
     *     functions defined by the user.
     *
     * @param xf Final state vector (n x 1)
     * @param tf Final time value
     *
     * @return Final cost
     */
    CostType final_cost(StateType xf, TimeType tf);

    /**
     * @brief Simulate the system forward or backward in time
     *
     * @details Simulates the evolution of the system but does not update the
     *     actual states of the system. To actually evolve the system use the
     *     `run` method.
     *
     * @param init_state Initial state vector (n x 1)
     * @param init_time Initial time
     * @param duration Time duration of simulation
     * @param dir Direction of time evolution
     * @return State at final time of simulation (n x 1)
     */
     Trajectory simulate(StateType init_state,
                         TimeType init_time,
                         TimeType duration,
                         System::Direction dir=System::FORWARD);




    //--------------------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------------------
    /**
     * @brief System dynamics function pointer.
     * @details This function maps from state, input, and time to the state
     * derivative.
     *
     * @param x State vector (n x 1)
     * @param u Input vector (m x 1)
     * @param t Time value
     * @return State derivative vector (n x 1)
     */
    function< StateType (StateType x, InputType u, TimeType t) > dynamics;

    /**
     * @brief System instantaneous time cost function pointer.
     * @details This function returns instantaneous cost for the given time
     *     value.
     *
     * @param t Time value
     * @return Cost value
     */
    function< CostType (TimeType t) > time_cost;

    /**
     * @brief System instantaneous state cost function pointer.
     * @details This function returns instantaneous cost for the given state
     *     value.
     *
     * @param x State vector (n x 1)
     * @return Cost value
     */
    function< CostType (StateType x) > state_cost;

    /**
     * @brief System instantaneous input cost function pointer.
     * @details This function returns instantaneous cost for the given input
     *     value.
     *
     * @param u Input vector (m x 1)
     * @return Cost value
     */
    function< CostType (InputType u) > input_cost;

    /**
     * @brief System final time cost function pointer.
     * @details This function returns final cost for the given time
     *     value.
     *
     * @param tf Final time value
     * @return Cost value
     */
    function< CostType (TimeType tf) > final_time_cost;

    /**
     * @brief System final state cost function pointer.
     * @details This function returns final cost for the given state
     *     value.
     *
     * @param xf Final state vector (n x 1)
     * @return Cost value
     */
    function< CostType (StateType xf) > final_state_cost;

    /**
     * @brief System controller function pointer.
     * @details This function maps from state and time to input.
     *
     * @param x State vector (n x 1)
     * @param t Time value
     *
     * @return Input vector (m x 1)
     */
    function< InputType (StateType x, TimeType t) > controller;

    /**
     * @brief System state constraint function pointer.
     * @details This functions constrains the state to a given set of values.
     *
     * @param x Unconstrained state vector (n x 1)
     * @return Constrained state vector (n x 1)
     */
    function< StateType (StateType x) > state_constraint;

    /**
     * @brief System input constraint function pointer.
     * @details This functions constrains the input to a given set of values.
     *
     * @param u Unconstrained input vector (m x 1)
     * @return Constrained input vector (m x 1)
     */
    function< InputType (InputType u) > input_constraint;


protected:
    //--------------------------------------------------------------------------
    // Protected Member Functions
    //--------------------------------------------------------------------------



    //--------------------------------------------------------------------------
    // Protected Member Variables
    //--------------------------------------------------------------------------



private:
    //--------------------------------------------------------------------------
    // Constants, Enums, and Types
    //--------------------------------------------------------------------------


    //--------------------------------------------------------------------------
    // Private Member Functions
    //--------------------------------------------------------------------------
    /**
     * @brief Outputs interpolated input between sample times.
     * @details
     *
     * @param u [description]
     * @return [description]
     */
    InputType input_interpolation(InputType u);


    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    // Code parameters
    TimeType init_step_factor_ = 1/10.0;
    InputInterpMethod input_interp_method = ZOH;

    // System parameters
    TimeType t_ = 0.0; // Current time
    TimeType ts_;      // Time step
    unsigned int n_;   // State dimension
    StateType x_;      // Current state
    unsigned int m_;   // Input dimension
    InputType u_;      // Input
    unsigned int l_;   // Output dimension
    OutputType y_;     // Output
    CostType L_;       // Instantaneous cost
    CostType cum_L_;   // Cumulative instantaneous cost
    CostType final_L_; // Final cost
    CostType J_;       // Total cost


}; // class
//------------------------------------------------------------------------------
// Postfix Increment Operators
//------------------------------------------------------------------------------

} // namespace optimal

#endif
