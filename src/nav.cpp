#ifndef NAV_CPP
#define NAV_CPP
#include <turtlebot/mymsg.h>
#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
// how much we need in velocity commands to move incrementAmt forward
const double movementMultiple = 1.8; 
// how much we move forward/backward each increment
const double incrementAmt = .1;
// how much angular velocity we need to move right 90 degrees
const double left_90 = 2.55;
// how much angular velocity we need to move right 90 degrees
const double right_90 = -2.56;
// this is how long the TurtleBot takes to move incrementAmt distance forward
const double movementInterval = 500000;



// Change the value of Movement multiple until turtlebot moves forward by incrementAmt.
// And stops for movementInterval.
void RoboState::testForward()
{
  // may need to adjust value for whatever reason
  usleep(movementInterval);
  double xMoveCommand; 
  // only move forward incrementAmt if the amount left to move is greater than incrementAmt

  // ideally, this should result in forward (or backward movement)
  // in the x direction by incrementAmt
  xMoveCommand = incrementAmt*movementMultiple;
  this->velocityCommand.linear.x = incrementAmt*movementMultiple;
  this->velocityCommand.angular.z = 0.0;
	  
  velocityPublisher.publish(this->velocityCommand);
  // ideally, this is the amount that x has changed
  // we should wait until forward movement has finished before we go on
  usleep(movementInterval);

  ROS_INFO("We moved %f", incrementAmt);
  

}

  // this is called whenever we receive a message 
void RoboState::messageCallback(const turtlebot::mymsg::ConstPtr& msg)
{
  //ROS_INFO("got packet: [%s]", msg->data.c_str());
  //ROS_INFO("got packet: ");
    if(!isMessageSet())
      {
	ROS_INFO("X and Y coordinates sent were: x:%f y:%f", msg->x, msg->y);
	setX(msg->x);
	setY(msg->y);
	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	setMessageStatus(true);
	setTurnAndGoForward(true);
      }
    else
      ROS_INFO("Cannot accept message. Movement still in progress.");

    /* cout << "X: "<< msg->x << std::endl;
  cout << "Y: " << msg->y << std::endl;
  cout << endl;
    */
}

void RoboState::turnForward()
{


  double angleRotation = 0;
    double radianConvert = 57.288;
    if (getX()>0 && getY()==0){
         angleRotation=0;
    }
    else if(getX()<0 && getY()==0){
        angleRotation=180;
    }
    else if(getX()==0 && getY()>0){
        angleRotation=90;
    }
    else if(getX()==0 && getY()<0){
        angleRotation=-90;
    }
    else if(getX()<0 && getY()>0){
      angleRotation=(180+(atan(getY()/getX())*radianConvert));
    }
    else if(getX()<0 && getY()<0){
      angleRotation=(-180+(atan(getY()/getX())*radianConvert));
    }
    else{
      angleRotation=(atan(getY()/getX())*radianConvert);
    }

 
   // gives the angle between y axis and hypotenuse

    // uses pythagorean theorem to figure out distance 
    double finalX = sqrt(pow(getX(),2) + pow(getY(),2));

    ROS_INFO("The angle of rotation is %f", angleRotation);
    
    this->velocityCommand.linear.x = 0.0;
    // positive angle of rotation, so we just rotate angleRotation/90 times the amount we would need to rotate 90 degrees left
    if(angleRotation > 0)
      this->velocityCommand.angular.z = left_90*(angleRotation/90);
    else // negative angle of rotation, so we just rotate angleRotation/90 times the amount we would need to rotate 90 degrees right
      this->velocityCommand.angular.z = -1*right_90*(angleRotation/90);
    velocityPublisher.publish(this->velocityCommand);
    // x is aligned with the hypotenuse, so we set it equal to the length of the hypotenuse
    setX(finalX);
    setY(0);

    // we adjusted the values, so we no longer need to turn
    setTurnAndGoForward(false);
    
    
  }
  
  void RoboState::bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
  {
    // if bumpers don't complain, don't run the loop
    if(msg->bumps_wheeldrops != 0){
    ROS_INFO("You hit an object! Motion terminating.");
    ROS_INFO("The remaining x was:%f and the remaining y was: %f.", getX(), getY());
    setX(0);
    setY(0);
    // allow RoboState to receive messages again
    setMessageStatus(false);
    }
  }
  
  // Basic initialization and publisher/subscribers. May need to adjust rates.
  RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), messageStatus(false), turnAndGoForward(false)
  {
    this->node = rosNode;
    this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    this->messageSubscriber= this->node.subscribe("my_msg", 1000, &RoboState::messageCallback, this);
    this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 100, &RoboState::bumperCallback, this);
  }

  // basically goForward does everything for us until it tells us we're done
  void RoboState::goRobotGo()
  {
    if(isMessageSet()){
      goForward();
    }
  }
  
  // first we turn the robot towards destination, then just go forward like normal
  void RoboState::turnThenForwardGo(){
    if(isMessageSet()){
      if(getTurnAndGoForward())
	turnForward();
      else
	goForward();
    }

  }

  bool RoboState::getTurnAndGoForward()
  {
    return turnAndGoForward;
  }
  
  double RoboState::xIsNegative()
  {

    if( getX() >= 0)
      return 1;
    else
      return -1;
  }

  void RoboState::goForward()
  {
    // may need to adjust value for whatever reason

    //    usleep(100000);

    
    if(getX()!=0)
      {

	double xMoveCommand; 
	// only move forward incrementAmt if the amount left to move is greater than incrementAmt
	if (std::abs(getX()) > incrementAmt){
	  // ideally, this should result in forward (or backward movement)
	  usleep(movementInterval);
	  // in the x direction by incrementAmt
	  xMoveCommand = incrementAmt*movementMultiple*xIsNegative();
	  this->velocityCommand.linear.x = xMoveCommand;
	  this->velocityCommand.angular.z = 0.0;
	  
	  velocityPublisher.publish(this->velocityCommand);
	  ROS_INFO("The amount we published was %f", xMoveCommand);
	  // ideally, this is the amount that x has changed
	  this->incrementX(-1*xIsNegative()*incrementAmt);
	  // we should wait until forward movement has finished before we go on
	  usleep(50000);

	  ROS_INFO("We moved %f", incrementAmt);
	  ROS_INFO("The remaining amount to move is %f", getX());
	}
	else{
	  // we have less than the incrementAmt left, so we move however much remaining
	  // do not need to know if xCoord is negative or not, since we get the actual value
	  xMoveCommand = getX()*movementMultiple;
	  this->velocityCommand.linear.x = xMoveCommand;
	  this->velocityCommand.angular.z = 0.0;

	  velocityPublisher.publish(this->velocityCommand);
	  ROS_INFO("We moved forward %f", getX());
	  // assume we moved forward or backward in the exactly how much was left in xCoord
	  this->incrementX(-getX());

	  ROS_INFO("The remaining amount to move is %f (should be zero)", getX());
	}
      }
    else{
      if(getY()==0){
	// means that both x and y are 0, so we should stop 
	// does this by setting messageStatus to false
	ROS_INFO("The both x and y are zero, so we are done with movemet.");
	ROS_INFO("Feel free to send more messages.");
	setMessageStatus(false);
      }
      else{
	// if x is zero, but y isnt, then we need to rotate to face towards the final destination
	rotateLR();
	ROS_INFO("The value of x was zero, so we did not move forward.");
      }
    }
  }
	      


  // assumes that x is zero and y is nonzero. May want to add cases for when method is called in error
  void RoboState::rotateLR()
  {
    ROS_INFO("Switching x and y coordinates.");
    usleep(500000);
    if( getY() > 0) // means that destination is on right
      rotateLeft();
    else if ( getY() < 0) // means that destination is on right
      rotateRight();
    else 
      setMessageStatus(false); // means that x and y was zero
    if(getY() > 0){
      setX(getY());
      ROS_INFO("Your y-coordinate was positive.");
    }
    else if(getY() < 0){
      setX(-getY());
      ROS_INFO("Your y-coordinate was negative.");
    }
      else{
	ROS_INFO("Your y-coordinate was zero.");
      }
      setY(0);
    ROS_INFO("x and y coordinates swapped.");
  }
  
  void RoboState::rotateLeft()
  {
    ROS_INFO("Rotating left.");
    usleep(500000);
    this->velocityCommand.linear.x = 0.0;
    this->velocityCommand.angular.z = left_90;
    
	
    velocityPublisher.publish(this->velocityCommand);
    usleep(100000);
  }

  void RoboState::rotateRight()
  {
    ROS_INFO("Rotating right.");
    usleep(500000);
    this->velocityCommand.linear.x = 0.0;
    this->velocityCommand.angular.z = right_90;
    velocityPublisher.publish(this->velocityCommand);
    usleep(100000);
  }


  void RoboState::setMessageStatus(bool status)
  {
    messageStatus=status;
  }

  bool RoboState::isMessageSet()
  {
    return messageStatus;
  }
  
  void RoboState::incrementX(double x)
  {
    xCoord += x;
  }

  void RoboState::incrementY(double y)
  {
    yCoord += y;
  }
  
  double RoboState::getX()
  {
    return xCoord;
  }

  void RoboState::setX(double x)
  {
    xCoord=x;
  }

  void RoboState::setY(double y)
  {
    yCoord=y;
  }

  double RoboState::getY()
  {
    return yCoord;
  }


  
  void RoboState::setTurnAndGoForward(bool value)
  {
    turnAndGoForward = value;
  }

#endif
