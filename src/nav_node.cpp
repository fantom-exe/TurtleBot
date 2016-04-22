#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>

  /* This is a visualization of what the x and y coordinates represent on 
   relative to the direction that the turtlebot is facing.
|         X+        . (destination)
|        |
|        |
|      (forward)
|        __
|      /   \
|      |___|    
|        ____________ Y-
  */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  //ros::Subscriber sub = nh.subscribe("my_msg", 1000, messageCallback);
  geometry_msgs::Twist cmd_vel;
  ros::Rate loopRate(1); // 10 hz

  //  while(ros::ok())
  // {
      // turnThenForward go is invoked when we want TurtleBot to turn in direction of destination
      // then go forward
      //robot.turnThenForwardGo();
      robot.testForward();
      // goRobotGo is invoked when we want to move forward, xCoord amount, then
      // rotate towards destination and then move forward
      //  robot.goRobotGo();
	    
      ros::spinOnce();
      loopRate.sleep();
      //    }
  return 0;
}




