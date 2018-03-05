/*
odom.h

Greg Hedger
4 March 2018

This common header facilitates communication between the ROS interface layer
and the underlying ROS-naive worker layer that interfaces directly with RS-232
*/


#pragma once

#include <string>

const int MAX_PORT_STR_SIZE = 32;

struct WorkerParams {
  int baud_rate_;
  std::string port_;
};




