/* ROS-CHOMP.
 *
 * Copyright (C) 2015 Jafar Qutteineh. All rights reserved.
 * License (3-Cluase BSD): https://github.com/j3sq/ROS-CHOMP/blob/master/LICENSE

* This code uses and is based on code from:
*   Project: trychomp https://github.com/poftwaresatent/trychomp
*   Copyright (C) 2014 Roland Philippsen. All rights reserved.
*   License (3-Clause BSD) : https://github.com/poftwaresatent/trychomp
/**
   \file chomp.hpp

   CHOMP for point vehicles (x,y) moving holonomously in the plane. It will
   plan a trajectory (xi) connecting start point (qs) to end point (qe) while
   avoiding obstacles (obs)
*/

#ifndef CHOMP_HPP
#define CHOMP_HPP

namespace chomp{
typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Isometry3d Transform;
void chomp(Vector const &qs,Vector const &qe, Vector &xi,Matrix const &obs);
}
#endif // CHOMP_HPP
