#include "ros/ros.h"
#include <sample_pubsub/mymsg.h>
#include <iostream>

using namespace std;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void mymsg_Callback(const sample_pubsub::mymsg::ConstPtr& msg)
{
  //ROS_INFO("got packet: [%s]", msg->data.c_str());
  //ROS_INFO("got packet: ");
  cout << "X: "<< msg->x << std::endl;
  cout << "Y: " << msg->y << std::endl;
  cout << endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("my_msg", 1000, mymsg_Callback);

  ros::spin();

  return 0;
}

