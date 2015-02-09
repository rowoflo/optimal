/**
 * @file system.h
 *
 * @brief System class header file
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
#include <functional>
#include <Eigen/Dense>
#include <vector>
#include <iostream>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace optimal
{


//------------------------------------------------------------------------------
// Typedefs
//------------------------------------------------------------------------------

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
        FORWARD=1,
        BACKWARD=-1
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
     * @param state_dim State dimension
     * @param input_dim Input dimension
     * @param output_dim Output dimension
     */
    System(unsigned int state_dim=1,
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
    void time(double t) { t_ = t; };
    double time() const { return t_; };
    void time_step(double ts) { ts_ = ts; };
    double time_step() const { return ts_; };
    void state_dim(unsigned int N) { N_ = N; };
    unsigned int state_dim() const { return N_; };
    void input_dim(unsigned int M) { M_ = M; };
    unsigned int input_dim() const { return M_; };
    void output_dim(unsigned int L) { L_ = L; };
    unsigned int ouput_dim() const { return L_; };
    void output(Eigen::VectorXd x) { x_ = x; };
    Eigen::VectorXd output() const { return x_; };


    //--------------------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------------------
    Eigen::VectorXd simulate(Eigen::VectorXd init_state,
                             double init_time,
                             double duration,
                             Direction dir=FORWARD);




    //--------------------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------------------
    /**
     * @brief System dynamics function pointer
     * @details This function maps from state, input, and time to the state
     * derivative.
     *
     * @param x State vector (N x 1)
     * @param u Input vector (M x 1)
     * @param t Time value
     * @return State derivative vector (N x 1)
     */
    std::function<Eigen::VectorXd
        (Eigen::VectorXd x, Eigen::VectorXd u, double t)> f;


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
    double t_;       // Current time
    double ts_;      // Time step
    unsigned int N_; // State dimension
    Eigen::VectorXd x_;     // Current state
    unsigned int M_; // Input dimension
    Eigen::VectorXd u_;     // Current input
    unsigned int L_; // Output dimension
    Eigen::VectorXd y_;     // Current output



}; // class
//------------------------------------------------------------------------------
// Postfix Increment Operators
//------------------------------------------------------------------------------

} // namespace optimal

#endif
