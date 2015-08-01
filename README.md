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

Usage:
------
```c++
void chomp(VectorXd const &qs, VectorXd const &qe, VectorXd &xi, MatrixXd const &obs);
```
* qs : Start point (x,y) as a 2x1 vector.
* qe : End point (x,y) as a 2x1 vector.
* xi : Generated chomp trajectory points (x<sub>1</sub>,y<sub>1</sub>,x<sub>2</sub>,y<sub>2</sub>,...,x<sub>N</sub>,y<sub>N</sub>) as 2*Nx1 vector. N is currently fixed to 20.
* obs : A matrix of disk like obstacles of size Kx3. Each row is (x,y,R) of the obstacle.

Notes:
* If the function is called with a non empty trajectory xi, the function will use the provided trajectory as an initial guess for the chomp algorithm.
* minimal_program.cpp shows a basic example of using chomp


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
