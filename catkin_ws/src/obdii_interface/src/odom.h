/*
odom.h

Greg Hedger
4 March 2018

This common header facilitates communication between the ROS interface layer
and the underlying ROS-naive worker layer that interfaces directly with RS-232
*/


#pragma once

#include <string>
#include <ros/ros.h>

// Constants

const int MAX_PORT_STR_SIZE = 32;

// Classes/Structures

struct WorkerParams {
  int baud_rate_;
  std::string port_;
  int polling_rate_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};


// Function prototypes

void *odom_thread(void *pv);

