#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel;	//define velocity to be controlled and published to cmd_velocity topic
float height;			//real height
double p;			//propotional gain
double i;			//integral gain
double d;			//derivative gain
float proE=0;			//propotional error
float proEP=0;			//propotional pervious error
float intE=0;			//integral error
float intE_max;			//the maximum integral error(avoiding increase until infinite)
float derE=0;			//derivative error
float pid;			//the output of PID function
float period;			//in seconds
double rate;			//in hertz
double desired_height;
void PidUpdate();


//this function will be triggered whenever a new data written to /sonar_height topic
void sonar_Callback(const sensor_msgs::Range::ConstPtr& msg){
   height = msg->range;
}

//the main function
int main(int argc, char **argv){
	
   //initializing ROS
   ros::init(argc, argv, "Height_PID_controller");
   ros::NodeHandle n2;
   
   //get the defined parameters in the launch file
   n2.getParam("p_gain", p);
   n2.getParam("i_gain", i);
   n2.getParam("d_gain", d);
   n2.getParam("intE_max", intE_max);
   n2.getParam("desired_height", desired_height);
   n2.getParam("rate", rate);

   //define the loop rate of the main function and calculate the period
   ros::Rate loop_rate(rate); 
   period = 1/rate;
   //ROS_INFO_STREAM("The system Period is :"<<period);

   //declare subscriber from /sonar_height topic
   ros::Subscriber IR_sub = n2.subscribe("/sonar_height", 100, &sonar_Callback);
   //declare publisher to /cmd_vel topic
   ros::Publisher Vel_pub = n2.advertise<geometry_msgs::Twist>("/cmd_vel",100);

   //the main loop
   while (ros::ok()){

	PidUpdate();		//call the pidUpdate function	
	vel.linear.z=pid;	//define the result of the pid function as linear velocity on the z axis

	Vel_pub.publish(vel);	//publish the new velocity to be applied by the motors
        ROS_INFO_STREAM("Desired height is :"<<desired_height); //print to terminal
	ROS_INFO_STREAM("Current height is :"<<height); 
	ros::spinOnce();	//increase the loop counter by one
	loop_rate.sleep();	//add delay according to the specified loop rate
		}
   //

return 0;
}

//the pid value Update function
void PidUpdate()
{
proEP=proE;	//save the last propotional error as pervious propotional error

proE=desired_height-height;	//calculate the new propotional error
derE=proE-proEP;		//calculate the derivative error
intE=intE+proE;			//calculate the integer error
if(intE>intE_max)intE=intE_max;	//limiting for the integer error

pid = (p*proE) + (i*intE*(period)) + (d*(derE/(period)));	//calculate the new PID function's output value
//ROS_INFO_STREAM("PID value is :"<<pid);
}
