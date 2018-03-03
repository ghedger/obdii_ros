/*
Greg Hedger
2 March 2018

This is a skeletal implementation of a node that will interface with the
Advanced Navigation OBDII Automotive Odometer to publish messages via ROS.

*/
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>   // Placeholder

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
public:
  ObdiiNode();

protected:
  void enableCallback(const std_msgs::BoolConstPtr& msg);
};

// ObdiiNode default constructor
ObdiiNode::ObdiiNode() :
  enable_(true)
{
  ros::param::get("~queue_size", queue_size_);
  ROS_INFO("OBDII OUTPUT QUEUE SIZE: %d", queue_size_);

  pub_ = nh_.advertise<nav_msgs::Odometry>("obdii_packet", 2);

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

  ros::spin();
}
