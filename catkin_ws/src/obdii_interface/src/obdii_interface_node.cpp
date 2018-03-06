/*
obdii_interface_node.cpp

Greg Hedger
2 March 2018

This is a skeletal implementation of a node that will interface with the
Advanced Navigation OBDII Automotive Odometer to publish messages via ROS.
*/

#include <boost/thread/mutex.hpp>
#include <pthread.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include "obdii_interface/ObdiiState.h"

#include "odom.h"

// ObdiiNode
// ROS Node to interface to OBDII device and
// publish salient data.
class ObdiiNode
{
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int queue_size_;
  bool enable_;
  ros::Subscriber enable_sub_;
  std::string pub_topic_;


  WorkerParams  workerParams_;

public:
  ObdiiNode();

  // Getters/Setters
  const WorkerParams *getWorkerParams() { return &workerParams_; }
  void setWorkerParams(const int baudRate, const int port) {
    workerParams_.baud_rate_ = baudRate;
    workerParams_.port_ = port;
  }

protected:
  void enableCallback(const std_msgs::BoolConstPtr& msg);
};

// ObdiiNode default constructor
ObdiiNode::ObdiiNode() :
  enable_(true)
{
  ros::param::get("~queue_size", queue_size_);
  ROS_INFO("OBDII OUTPUT QUEUE SIZE: %d", queue_size_);

  ros::param::get("~rs232_port", workerParams_.port_);
  ROS_INFO("OBDII RS232 PORT: %s", workerParams_.port_.c_str());

  ros::param::get("~rs232_baud_rate", workerParams_.baud_rate_);
  ROS_INFO("OBDII RS232 BAUD RATE: %d", workerParams_.baud_rate_);

  ros::param::get("~obdii_polling_rate", workerParams_.polling_rate_);
  ROS_INFO("OBDII RS232 POLLING RATE (Hz): %d", workerParams_.polling_rate_);

  ros::param::get("~pub_topic", pub_topic_);
  ROS_INFO("PUBLICATION TOPIC: %s", pub_topic_.c_str());

  workerParams_.nh_ = nh_;

  pub_ = nh_.advertise<obdii_interface::ObdiiState>(pub_topic_, 2);
  workerParams_.pub_ = pub_;

  ros::param::get("~enable", enable_);
  enable_sub_ = nh_.subscribe("enable", 1, &ObdiiNode::enableCallback, this);
}

// enableCallback
// Enable/disable publication, called on subscription
// Entry: std_msgs:BoolConstPtr
// Exit:  -
void ObdiiNode::enableCallback(const std_msgs::BoolConstPtr& msg)
{
  enable_ = msg->data;
}


// Main entry point
int main(int argc, char** argv)
{
  ros::init(argc, argv, "obdii_node");
  ObdiiNode* obdiiNode = new ObdiiNode();

  pthread_t workerThread;
  int ret = pthread_create(
    &workerThread,
    NULL,
    &odom_thread,
    (void *) obdiiNode->getWorkerParams()
  );
  if(ret) {
    ROS_ERROR("%s Unable to start worker thread", __FILE__);
    return -1;
  }
  ROS_INFO("odbii_interface_node RUNNING...");

  ros::spin();
}
