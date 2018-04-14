#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include "Graduation_Trakya/graph.h"
#include <ros/time.h>
#include <std_msgs/Empty.h>
#define GRAPH_DIMENSION 12
#define num_of_uav 3


int main(int argc, char** argv){ 
    
  // ROS Setup:
  ros::init(argc, argv, "Current_yaw");
  ros::NodeHandle n;


  ros::Publisher go_to_pub = n.advertise<geometry_msgs::Point>("/uav1/go_to",100);
  geometry_msgs::Point goal;
  ros::Rate loop_rate(50);

  while(ros::ok()){
      goal.x=0.0;
      goal.y=0.0;
      goal.z=2.0; 
ROS_INFO("Robot could not reach goal. Gave up!");
 go_to_pub.publish(goal);

    ros::spinOnce(); //needed to make sure that all your callbacks are triggered.
    loop_rate.sleep();
  }
  

  return 0;
}
