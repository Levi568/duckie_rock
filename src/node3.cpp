#include "ros/ros.h"

#include <iostream>
#include <duckie_rock/Point.h>

using namespace std;

void callback(const duckie_rock::Point::ConstPtr& msg){
  ROS_INFO("Detected obstacle [%s] at position [%f],[%f],[%f]",msg->id.c_str(),msg->x,msg->y,msg->z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node3");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/registered_obstacles", 1, callback);
  ros::spin(); //without this line, callback can not be established
  return 0;
}
