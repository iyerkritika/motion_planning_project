
#include "Astar.cpp"
#include "ros/ros.h"
// #include "std_msgs/String.h"
int main(int argc, char const *argv[])
{
  ros::init(argc, argv, "env_pass");
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  return 0;
}
