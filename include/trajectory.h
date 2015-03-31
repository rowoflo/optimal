/**
 * @file trajectory.h
 *
 * @brief Trajectory class header file.
 *
 * @author <a href="http://rowlandoflaherty.com">Rowland O'Flaherty</a>
 *
 * @date 2015 FEB 09
 *
 * @copyright Copyright (C) 2015, see LICENSE file
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_


//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <Eigen/Dense>
#include <vector>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

namespace optimal
{

//------------------------------------------------------------------------------
// Typedefs
//------------------------------------------------------------------------------
typedef double TimeType;
typedef VectorXd StateType;
typedef VectorXd InputType;
typedef VectorXd OutputType;
typedef double CostType;


/**
* @class Trajectory
*
* @brief A container class for storing dynamical system trajectories.
*
* TODO: Detailed description
*/
class Trajectory
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



    //--------------------------------------------------------------------------
    // Public Nested Classes
    //--------------------------------------------------------------------------



    //--------------------------------------------------------------------------
    // Lifecycle
    //--------------------------------------------------------------------------
    // Constructors
    Trajectory(unsigned int state_dim=1,
               unsigned int input_dim=0);


    // Destructors


    // Copy constructor


    //--------------------------------------------------------------------------
    // Overloaded Operators
    //--------------------------------------------------------------------------
    friend ostream& operator<< (ostream &out, Trajectory &traj);

    //--------------------------------------------------------------------------
    // Getters and Setters
    //--------------------------------------------------------------------------
    TimeType time(unsigned int i) const { return time_tape_[i]; };
    unsigned int state_dim() const { return n_; };
    StateType state(unsigned int i) const { return state_tape_[i]; };
    unsigned int input_dim() const { return m_; };
    InputType input(unsigned int i) const { return input_tape_[i]; };
    CostType cost(unsigned int i) const { return cost_tape_[i]; };

    //--------------------------------------------------------------------------
    // Public Member Functions
    //--------------------------------------------------------------------------
    unsigned int size() const { return time_tape_.size(); };
    void reserve(unsigned int n);
    void push_back(TimeType t, StateType x);
    void push_back(TimeType t, StateType x, InputType u);
    void push_back(TimeType t, StateType x, InputType u, CostType J);


    //--------------------------------------------------------------------------
    // Public Member Variables
    //--------------------------------------------------------------------------


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
    vector<TimeType> time_tape_;   // Time tape
    unsigned int n_;               // State dimension
    vector<StateType> state_tape_; // State tape
    unsigned int m_;               // Input dimension
    vector<InputType> input_tape_; // Input tape
    vector<CostType> cost_tape_;   // Total cost tape

}; // class
//------------------------------------------------------------------------------
// Postfix Increment Operators
//------------------------------------------------------------------------------

} // namespace optimal

#endif
