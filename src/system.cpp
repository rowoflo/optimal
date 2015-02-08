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
System::System(unsigned int N, unsigned int M, unsigned int L)
:
N_(N),
M_(M),
L_(L)
{
    f = [=] (Eigen::VectorXd x, Eigen::VectorXd u, double t)
    {
        return Eigen::VectorXd::Zero(N);
    };
}

// Destructor


// Copy constructor



// Copy helper function



//------------------------------------------------------------------------------
// Overloaded Operators
//------------------------------------------------------------------------------



//--------------------------------------------------------------------------
// Getters and Setters
//--------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Public Member Functions
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Protected Member Functions
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Private Member Functions
//------------------------------------------------------------------------------


} // namespace optimal
