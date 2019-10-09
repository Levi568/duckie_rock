#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>
#include <duckie_rock/Point.h>

using namespace std;
ros::Publisher pub;

duckie_rock::Point point;
int counter = 0;
std::stringstream ss;

void callback(const geometry_msgs::Point::ConstPtr& msg){
  counter++;
  point.id = to_string(counter);
  point.x = msg->x;
  point.y = msg->y;
  point.z = msg->z;

  pub.publish(point);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/obstacles_detected", 1, callback);
  pub = n.advertise<duckie_rock::Point>("/registered_obstacles", 1);
  ros::spin();
  return 0;
}
