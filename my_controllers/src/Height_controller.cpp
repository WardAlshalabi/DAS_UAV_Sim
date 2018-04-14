#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel;
float height;
const float desired_height=1.5;

void sonar_Callback(const sensor_msgs::Range::ConstPtr& msg){
   height = msg->range;
	
}

int main(int argc, char **argv){

   ros::init(argc, argv, "Height_controller");
   ros::NodeHandle n2;

   ros::Rate loop_rate(5); 

   ros::Subscriber IR_sub = n2.subscribe("/sonar_height", 100, &sonar_Callback);
   ros::Publisher Vel_pub = n2.advertise<geometry_msgs::Twist>("/cmd_vel",100);

   while (ros::ok()){
if (height < desired_height-0.4)
{
vel.linear.z=0.3;
ROS_INFO_STREAM("Increasing Altitude,Current Hight is:"<<height); 
}

else if (height > desired_height+0.4)
{
vel.linear.z=-0.3;
ROS_INFO_STREAM("Decreasing Altitude,Current Hight is:"<<height);
}

else if (height > desired_height+0.1)
{
vel.linear.z=-0.1;
ROS_INFO_STREAM("Decreasing Altitude,Current Hight is:"<<height);
}

else if (height < desired_height-0.1)
{
vel.linear.z=0.1;
ROS_INFO_STREAM("Decreasing Altitude,Current Hight is:"<<height);
}

else
{
vel.linear.z=0.0;
ROS_INFO_STREAM("Current Hight is:"<<height);
} 


Vel_pub.publish(vel);
ros::spinOnce();
loop_rate.sleep();
}
return 0;
}
