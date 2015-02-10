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
typedef dense_output_runge_kutta
    <controlled_runge_kutta< runge_kutta_dopri5< StateType > > > Stepper;

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
    void state_dim(unsigned int N) { N_ = N; };
    unsigned int state_dim() const { return N_; };
    void input_dim(unsigned int M) { M_ = M; };
    unsigned int input_dim() const { return M_; };
    void output_dim(unsigned int L) { L_ = L; };
    unsigned int ouput_dim() const { return L_; };


    //--------------------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------------------
    /**
     * @brief Obtain input from state and time.
     * @details Outputs system input based on state and time using the current
     *     controller and input constraints functions defined by the user.
     *
     * @param x State vector (N x 1)
     * @param t Time value
     *
     * @return Input vector (M x 1)
     */
    InputType input(StateType x, TimeType t);

    /**
     * @brief Simulate the system forward or backward in time
     *
     * @details Simulates the evolution of the system but does not update the
     *     actual states of the system. To actually evolve the system use the
     *     `run` method.
     *
     * @param init_state Initial state vector (N x 1)
     * @param init_time Initial time
     * @param duration Time duration of simulation
     * @param dir Direction of time evolution
     * @return State at final time of simulation (N x 1)
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
     * @param x State vector (N x 1)
     * @param u Input vector (M x 1)
     * @param t Time value
     * @return State derivative vector (N x 1)
     */
    function< StateType (StateType x, VectorXd u, TimeType t) > dynamics;

    /**
     * @brief System controller function pointer.
     * @details This function maps from state and time to input.
     *
     * @param x State vector (N x 1)
     * @param t Time value
     *
     * @return Input vector (M x 1)
     */
    function< InputType (StateType x, TimeType t) > controller;

    /**
     * @brief System input constraint function pointer.
     * @details This functions constraints the input a given set of values.
     *
     * @param u Unconstrained input vector (M x 1)
     * @return Constrained input vector (M x 1)
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



    //--------------------------------------------------------------------------
    // Private Member Variables
    //--------------------------------------------------------------------------
    // Code parameters
    TimeType init_step_factor_ = 1/10.0;

    // System parameters
    TimeType t_ = 0.0; // Current time
    TimeType ts_;      // Time step
    unsigned int N_;   // State dimension
    StateType x_;      // Current state
    unsigned int M_;   // Input dimension
    InputType u_;      // Current input
    unsigned int L_;   // Output dimension
    OutputType y_;     // Current output


}; // class
//------------------------------------------------------------------------------
// Postfix Increment Operators
//------------------------------------------------------------------------------

} // namespace optimal

#endif
