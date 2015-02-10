//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "trajectory.h"

#include <iostream>
#include <boost/format.hpp>
#include <boost/numeric/odeint.hpp>

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
Trajectory::Trajectory(unsigned int state_dim,
                       unsigned int input_dim)
:
N_(state_dim),
M_(input_dim)
{

}


// Destructor


// Copy constructor



// Copy helper function



//------------------------------------------------------------------------------
// Overloaded Operators
//------------------------------------------------------------------------------
std::ostream& operator<< (std::ostream &out, Trajectory &traj)
{
    TimeType t;
    StateType x;
    InputType u;

    cout << boost::format("%1$=10s  |  %2$=30s  |  %3$=10s  |\n")
                          % "Time" % "State" % "Input";

    for (int i=0; i<traj.time_tape_.size(); ++i) {
        t = traj.time_tape_[i];
        x = traj.state_tape_[i];
        u = traj.input_tape_[i];

        cout << boost::format("%1$-10.3f  |  %2$-30.3f  |  %3$-10.3f  |\n")
                % t % x.transpose() % u.transpose();
        out << "\n";
    }
    return out;
}


//------------------------------------------------------------------------------
// Getters and Setters
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Public Member Functions
//------------------------------------------------------------------------------
void Trajectory::push_back(TimeType t, StateType x)
{
    InputType u;
    u = u.setZero(M_)*NAN;
    push_back(t, x, u);
}

void Trajectory::push_back(TimeType t, StateType x, InputType u)
{
    time_tape_.push_back(t);
    state_tape_.push_back(x);
    input_tape_.push_back(u);
}


//------------------------------------------------------------------------------
// Protected Member Functions
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Private Member Functions
//------------------------------------------------------------------------------



// #include <boost/iterator/zip_iterator.hpp>
// #include <boost/range.hpp>

// template <typename... T>
// auto zip(const T&... containers) -> boost::iterator_range<boost::zip_iterator<decltype(boost::make_tuple(std::begin(containers)...))>>
// {
//     auto zip_begin = boost::make_zip_iterator(boost::make_tuple(std::begin(containers)...));
//     auto zip_end = boost::make_zip_iterator(boost::make_tuple(std::end(containers)...));
//     return boost::make_iterator_range(zip_begin, zip_end);
// }

} // namespace optimal
