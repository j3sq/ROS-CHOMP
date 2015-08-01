/* ROS-CHOMP.
 *
 * Copyright (C) 2015 Jafar Qutteineh. All rights reserved.
 * License (3-Cluase BSD): https://github.com/j3sq/ROS-CHOMP/blob/master/LICENSE
 *
 * **
 * \file ros_chomp.cpp
 *
 * ROS support for chomp path adaptor for Cargo-ANTS project http://cargo-ants.eu/
 *
 *
 */
#include <iostream>
#include <Eigen/Dense>
#include "chomp.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace Eigen;
using namespace std;
typedef VectorXd Vector;
typedef MatrixXd Matrix;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "chomp_listner");
  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("chatter", 1000, chatterCallback);

  ros::spin();
  return 0;
	// VectorXd qs(2);  //Start goal  coordinates (x,y)
	// VectorXd qe(2);  //End goal  coordinates (x,y)
	// VectorXd xi;     //Trajectory points (x0,y0,x1,y1,....)
	// MatrixXd obs;    //obstacles |x0,y0,R0;x1,y1,R1...|
  //
	// qs << 0, 0;
	// qe << 3, 5;
  //
  //
	// generatePath(qs, qe, xi, obs);
	// cout << xi << std::endl;
}
