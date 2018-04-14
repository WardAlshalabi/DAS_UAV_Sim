//Drone Controller make it flying between sepecified series of points

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

int index1=0;
double path[4][3]={
    {10,10,1},
    {-10,10,2},
    {-10,-10,2},
    {10,-10,2}
};

//Declarations//
geometry_msgs::Twist vel;           //define twist type variable to save velocities inside
geometry_msgs::Point current_Pos;   //point type variable to save coordinates of current position
geometry_msgs::Point desired_Pos;   //point type variable to save coordinates of desired position
double distance_tolerance=0.5;      //tolerance in distance between current and desired positions
double angle_tolerance=15;
tf::Pose current_orientation;       //Orientation of robot in quaternion system
double roll, pitch,current_yaw;                 //yaw of robot in radyan
double RAD2DEG = 57.2957795130823209;
double TWOPI = 6.2831853071795865;
double desired_yaw=0.0;
double OriError=0.0;

double rate=20;                     //Loop rate in HZ
double period;			            //Period in seconds
bool b=false;
////

//PID's variables//
//                  Linear_velocity     Angular_velocity     Height
double p[3]=        {0.4,               0.2,                    1.0};    //propotional gains
double i[3]=        {0.0,               0.04,                  0.2};	//integral gains
double d[3]=        {0.0,               0.04,                     0.2};	//derivative gains
double intE_max[3]= {5,                 1,                     0.2};	//the maximum integral error(avoiding increase until infinite)
double proE[3];			//propotional errors
double proEP[3];		//propotional pervious errors
double intE[3];			//integral errors
double derE[3];			//derivative errors
double PidUpdate(int a);//declaring PID function
double UpdateDesiredYaw();
double ErrorInOri();
//

//Function calculates distance between two points
double getDistance(double x1,double x2,double y1,double y2,double z1=0.0,double z2=0.0)
{
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));
}
//

//Callback function will be triggered each time something published to /ground_truth/state topic
void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){
   current_Pos.x = msg->pose.pose.position.x;   //get current x Position
   current_Pos.y = msg->pose.pose.position.y;   //get current Y Position
   current_Pos.z = msg->pose.pose.position.z;   //get current Z Position
   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll, pitch, current_yaw);
   //tf::poseMsgToTF(msg->pose.pose, current_orientation);  //get current orientation as quaternion
   //current_yaw = tf::getYaw(current_orientation.getRotation()); //calculate yaw in radyans from quaternion orientation
   if(current_yaw<0)current_yaw=3.14159+(current_yaw+3.14159);
   current_yaw=current_yaw*RAD2DEG;
   if(current_yaw<90)current_yaw=90-current_yaw;
   else current_yaw=450-current_yaw;
   //ROS_INFO("Yaw: [%f]",current_yaw);
}

//Callback function will be triggered each time something published to /go_to topic
void go_to_Callback(const geometry_msgs::Point::ConstPtr& msg){
  desired_Pos.x = msg->x;   //get desired x Position
  desired_Pos.y = msg->y;   //get desired y Position
  desired_Pos.z = msg->z;   //get desired z Position
  b=true;
}

//Callback function will be triggered each time something published to /sonar_height topic
void sonar_Callback(const sensor_msgs::Range::ConstPtr& msg){
  // current_Pos.z = msg->range;
}

//**The main Loop**//
int main(int argc, char **argv){
   
    //initializing ROS
    ros::init(argc, argv, "GoToPosition2");
    ros::NodeHandle n;
    ros::Rate loop_rate(rate);
    period = 1/rate;    //calculate Period according to the main loop frequency
    //

    //Subscribers and Publishers Declarations
    ros::Subscriber IR_sub = n.subscribe("/sonar_height", 100, &sonar_Callback);    //declaration of /sonar_height topic subscriber
    ros::Subscriber go_to_sub = n.subscribe("/go_to", 100, &go_to_Callback);        //declaration of /go_to topic subscriber
    ros::Subscriber Odom_sub = n.subscribe("/ground_truth/state", 100, &Odom_Callback); //declaration of /ground_truth/state topic subscriber
    ros::Publisher Vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);     //declaration of subscriber to /cmd_vel topic
    //

    //if there is no desired position received at launch take at as (0,0,0)
    current_Pos.x=0;
    current_Pos.y=0;
    current_Pos.z=0;

    /*    
    std::cout<<"Enter the x Position:";
    std::cin>>desired_Pos.x;
    std::cout<<"Enter the Y Position:";
    std::cin>>desired_Pos.y;
    std::cout<<"Enter the Z Position:";
    std::cin>>desired_Pos.z;
    */

    while (ros::ok()){

    if(getDistance(current_Pos.x,desired_Pos.x,current_Pos.y,desired_Pos.y,current_Pos.z,desired_Pos.z)>distance_tolerance)
     {
    vel.linear.x=PidUpdate(0);
    vel.linear.z=PidUpdate(2);
    vel.angular.z=PidUpdate(1);
	Vel_pub.publish(vel);	//publish the new velocity to be applied on the Drone   
     }
  
    else
		{
    vel.linear.x=0;
    vel.linear.y=0;
    vel.linear.z=0;	
    vel.angular.x=0;
    vel.angular.y=0;
    vel.angular.z=0;
	Vel_pub.publish(vel);
    }

    ros::spinOnce();	//increase the loop counter by one
	loop_rate.sleep();	//add delay according to the specified loop rate 
     }
    return 0;
     }

double UpdateDesiredYaw()
{
    desired_yaw=atan2((desired_Pos.x-current_Pos.x),(desired_Pos.y-current_Pos.y));
if (desired_yaw < 0.0) desired_yaw =desired_yaw+ TWOPI;
   desired_yaw=RAD2DEG * desired_yaw;
   return desired_yaw;
}

double ErrorInOri()
{
    if(current_yaw<UpdateDesiredYaw())
    {
        if((UpdateDesiredYaw()-current_yaw)>180) OriError=(360-(UpdateDesiredYaw()-current_yaw));
        else OriError=-1*(UpdateDesiredYaw()-current_yaw);
    }
    else
    {
        if((current_yaw-UpdateDesiredYaw())<180) OriError=(current_yaw-UpdateDesiredYaw());
        else OriError=-1*(((360-current_yaw)+UpdateDesiredYaw()));
    }
    return OriError;
}

// The PID Update function //
double PidUpdate(int a)
{
proEP[a]=proE[a];	//save the last propotional error as pervious propotional error
//calculate the new propotional error
switch(a) 
{
case 0: //The PID value will be calculated for Linear velocity
    proE[a]=getDistance(current_Pos.x,desired_Pos.x,current_Pos.y,desired_Pos.y);	
    break;

case 1: //The PID value will be calculated for Angular velocity
    proE[a]=ErrorInOri();
    break;

case 2: //The PID value will be calculated for Height
    proE[a]=desired_Pos.z-current_Pos.z;
    break;
}
//
derE[a]=proE[a]-proEP[a];		//calculate the derivative error
intE[a]=intE[a]+proE[a];			//calculate the integer error
if(intE[a]>intE_max[a])intE[a]=intE_max[a];	//limiting for the integer error
return (p[a]*proE[a]) + (i[a]*intE[a]*(period)) + (d[a]*(derE[a]/(period)));	//calculate the new PID function's output value
}
////////