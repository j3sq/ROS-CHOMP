ROS-CHOMP
=================

Based on source code available at [trychomp][]

Build and Run GUI:
-------
You need [CMake][], [Eigen][] v3, and [GTK+][] v2.

    git clone https://github.com/j3sq/ROS-CHOMP.git chomp
    mkdir chomp/build
    cd chomp/build
    cmake ..
    make
    ./chomp-shell

Minimal Program:
------
```c++
#include <iostream>
#include <Eigen/Dense>
#include "chomp.hpp"

int main()
{
  Eigen::VectorXd qs(2); //Start goal  coordinates (x,y)
  Eigen::VectorXd qe(2); //End goal  coordinates (x,y)
  Eigen::VectorXd xi; //Trajectory points (x0,y0,x1,y1,....)
  Eigen::MatrixXd obs; //obstacles |x0,y0,R0|
                      //           |x1,y1,R1|
                      //           | .......|
  qs<<0,0;
  qe<<3,5;

  chomp::chomp(qs,qe,xi,obs);
  std::cout<<xi<<std::endl;
}
```


Changes from [trychomp][]:
-------------------------
* Separation of GUI implementation from chomp implementation. chomp.cpp is now a library style function with no GUI code attached. chomp-shell.cpp is GUI only implementation with no actual chomp implementation.
* Added Iteration limit and early termination criteria.
* Obstacles (previously repulsors) are now handled via Matrix object (instead of custom handle_s structure).
* chomp.cpp is not limited to 2 obstacles. In chomp-shell right clicking will add more obstacles.
* If no initial trajectory *xi* is provided, chomp will use line of sight as initial guess. This considerably drops down the number of iterations to reach a solution

To Do:
------
* Add support for other vehicles
* Add ROS support
* Enforce NH motion model

As the name implies, this is based on [CHOMP][].

[CMake]: http://cmake.org/
[Eigen]: http://eigen.tuxfamily.org/
[GTK+]: http://www.gtk.org/
[CHOMP]: http://www.nathanratliff.com/research/chomp
[trychomp]: https://github.com/poftwaresatent/trychomp
