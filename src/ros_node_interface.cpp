//#include "Astar.cpp"
//#include "rrtstar.cpp"
#include "arastar.cpp"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/MarkerArray.h"
#include <math.h>

using namespace std;

std::vector<float> start,goal;/*
std::vector<std::vector<float> > path_rrt;
 std::vector<std::vector<float> > path_astar;
 std::vector<std::vector<float> > path_arastar;

// Astar_Node* g;

bool get = false;*/
void call_back(const nav_msgs::OdometryConstPtr msg)
{
    start.clear();
    start.push_back(floorf(msg->pose.pose.position.x*1000)/1000);
    start.push_back(floorf(msg->pose.pose.position.y*1000)/1000);
    start.push_back(floorf(msg->pose.pose.orientation.z*1000)/1000);
//    ROS_INFO("in odom");
//    get = true;
}
/*
visualization_msgs::MarkerArray path_draw()
{
//   cout<<"lalalalala\n";
    visualization_msgs::MarkerArray marker_array_msg;
    std::vector<float> joint_values;

//     for (unsigned int i = 0; i < path_astar.size(); i++)
//     for (unsigned int i = 0; i < path_arastar.size(); i++)
    for (unsigned int i = 0; i < path_rrt.size(); i++)
    {
//                ROS_INFO("l %d\n",i);
//                 joint_values.push_back(path_astar[i][0]);
//                 joint_values.push_back(path_astar[i][1]);
//                 joint_values.push_back(path_astar[i][2]);
//                 joint_values.push_back(path_arastar[i][0]);
//                 joint_values.push_back(path_arastar[i][1]);
//                 joint_values.push_back(path_arastar[i][2]);
//                joint_values.push_back(path_rrt[i][0]);
//                joint_values.push_back(path_rrt[i][1]);
//                joint_values.push_back(path_rrt[i][2]);

      //          geometry_msgs::Point p;

      //          p.x = end_effector_state.translation()[0];
      //          p.y = end_effector_state.translation()[1];
      //          p.z = end_effector_state.translation()[2];

      //          ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
      //          ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());


//                 marker_array_msg.markers.resize(path_astar.size());
//                 marker_array_msg.markers.resize(path_arastar.size());
//                marker_array_msg.markers.resize(path_rrt.size());
                // ROS_INFO("Hello I am here");
                marker_array_msg.markers[i].header.frame_id = "chassis";
                marker_array_msg.markers[i].header.stamp = ros::Time();
//                marker_array_msg.markers[i].ns = "ExpandedStates";
                marker_array_msg.markers[i].id = i;
                marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE;
                marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
                marker_array_msg.markers[i].pose.position.x = joint_values[0];
                marker_array_msg.markers[i].pose.position.y = joint_values[1];
                marker_array_msg.markers[i].pose.position.z = 0;
                marker_array_msg.markers[i].pose.orientation.x = 0.0;
                marker_array_msg.markers[i].pose.orientation.y = 0.0;
                marker_array_msg.markers[i].pose.orientation.z = 0.0;
                marker_array_msg.markers[i].pose.orientation.w = 1.0;
                marker_array_msg.markers[i].scale.x = 0.1;
                marker_array_msg.markers[i].scale.y = 0.1;
                marker_array_msg.markers[i].scale.z = 0.1;
                marker_array_msg.markers[i].color.a = 1.0;
                marker_array_msg.markers[i].color.r = 1.0;
                marker_array_msg.markers[i].color.g = 1.0;//change for rrt
                marker_array_msg.markers[i].color.b = 0.0;//for arasstar
      //        marker_array_msg.markers[i].lifetime = ros::Duration(0.1);
                joint_values.clear();

              }
       return marker_array_msg;
}*/



int main(int argc, char** argv)
{
//  cout<<"startin\n";
  ros::init(argc ,argv, "env_pass");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom",1,call_back);
  ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("/path_points",100);
  while(ros::ok())
  {
      ros::spinOnce();
//      if (get)
          break;
  }
//  cout<<"hi\n";
//  for (unsigned int i=0;i<start.size();i++)
//      cout<<start[i]<<"\n";
//  start.clear();
//  start.push_back(0);
//  start.push_back(0);
//  start.push_back(0);
//  cout<<"\n";
//  goal.push_back(5);
//  goal.push_back(1.2);
//  goal.push_back(M_PI/2);
//    printing();
    ROS_INFO("yolo");
//  path_rrt=RRTpath(start,goal);
//  g=Astar(start,goal);
//  path_astar=path(g);
//  path_arastar=ARAstar(start,goal);
  visualization_msgs::MarkerArray marker_array_msg;
//  marker_array_msg=path_draw();
  while(ros::ok())
  {
      pub.publish(marker_array_msg);
      ros::spinOnce();
  }
  return 0;
}
