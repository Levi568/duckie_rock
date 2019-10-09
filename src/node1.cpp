#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cstdlib>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node1");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Point>("/obstacles_detected", 1);
  ros::Rate loop_rate(10);
  srand (static_cast <unsigned> (time(0)));

  while (ros::ok())
  {
    geometry_msgs::Point msg;
    //generate random number in [0,1] for point x,y,z
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float s = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float t = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    //assign random numbers to points x,y,z
    msg.x = r;
    msg.y = s;
    msg.z = t;

    //std::stringstream ss;
    cout<<"x position : "<<msg.x<<" "
        <<"y position : "<<msg.y<<" "
        <<"z position : "<<msg.z<<" "<< endl;
    //msg.data = ss.str();

    //publish the geometry message
    pub.publish(msg);
    ros::spinOnce();
  }
  return 0;
}
