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
#include <Eigen/Dense>

#include <iostream>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace optimal;
using namespace Eigen;

//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------

VectorXd hFunc();

int main(int argc, char *argv[])
{
    System sys;

    Vector3d x;
    Vector2d u;
    double t;

    // sys.f = hFunc;
    cout << sys.f(x,u,t) << endl;
    // sys.f = [] () { cout << "Good bye" << endl; };
    // sys.f();
    // VectorXd m(3);
    // m << 1, 2, 3;
    // cout << m << endl;
}

VectorXd hFunc() {
    Vector3d m;
    m << 1, 2, 3;
    return m;
}
