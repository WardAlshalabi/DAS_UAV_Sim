//UAV Controller built for achieving autonomous flights to a specific point can be given by publishing to "/go_to" topic.
//or by calling "go2point" service.

//the controller can be turned on or off by calling "controller_state_srv" service.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include "Graduation_Trakya/graph.h"
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include "my_controllers/go2point.h"
#include "my_controllers/goalReached.h"
#include "my_controllers/controller_state_srv.h"
#include <cstdlib>
#define GRAPH_DIMENSION 15  ////remember to edit this number of you change the arena and the number of points
#define num_of_uav 3

//Declarations//
geometry_msgs::Twist vel;           //define twist type variable to save velocities inside
geometry_msgs::Point current_Pos;   //point type variable to save coordinates of current position
geometry_msgs::Point desired_Pos;   //point type variable to save coordinates of desired position
double distance_tolerance=0.2;      //tolerance in distance between current and desired positions
double angle_tolerance=20;          //Angle tolerance in degree
tf::Pose current_orientation;       //Orientation of robot in quaternion system
double roll, pitch,current_yaw;                 //roll,pitch,yaw of robot in radyan
double RAD2DEG = 57.2957795130823209;
double TWOPI = 6.2831853071795865;
double desired_yaw=0.0;             //the calculated desired yaw of the drone
double OriError=0.0;                //error in orientation between the desired yaw and the current one 
                                    //calculated continously with direction "Right or left".
int uav_id;                         //ID of Drone
double station_x;
double station_y;
double station_z;
std_msgs::Empty Empty_msg;
bool goal_re=false;
bool goal_re_srv=true;              //reaching the goal has been reported to the service client
double rate=50;                     //Loop rate in HZ
double period;			            //Period in seconds
bool b=false;                       //A flag enabled whenever a new position published to "/go_to" Topic.
bool control_is_on=true;        //when this flag turned off the program will run with empty loop so the controller will be shutdown
////

//PID's variables//
//                  Linear_velocity     Angular_velocity     Height
double p[3]=        {0.5,               0.2,                    1.0};    //propotional gains
double i[3]=        {0.0,               0.04,                  0.2};	//integral gains
double d[3]=        {0.0,               0.04,                     0.2};	//derivative gains
double intE_max[3]= {5,                 1,                     0.2};	//the maximum integral error(avoiding increase until infinite)
double proE[3];			//propotional errors
double proEP[3];		//propotional pervious errors
double intE[3];			//integral errors
double derE[3];			//derivative errors
double PidUpdate(int a);//declaring PID function
double UpdateDesiredYaw();//Function continously calculating the desired yaw in order to reach the target point
double ErrorInOri();//Function calculates the error in drone orientation and specify the sign of the error"Right or Left"
//

//Function calculates distance between two points
double getDistance(double x1,double x2,double y1,double y2,double z1=0.0,double z2=0.0)
{
    return sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));
}
//

//Callback function will be triggered each time something published to /ground_truth/state topic
//*** the "/ground_truth/state" topic is published by the simulator so, in the real life it's the topic where the sensores publish the position and the orientation of the drone to.
void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){

    //Get the position information
   current_Pos.x = msg->pose.pose.position.x;   //get current x Position
   current_Pos.y = msg->pose.pose.position.y;   //get current Y Position
   current_Pos.z = msg->pose.pose.position.z;   //get current Z Position

   //Get the orientation information in quaternion type
   tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   m.getRPY(roll, pitch, current_yaw);
   //tf::poseMsgToTF(msg->pose.pose, current_orientation);  //get current orientation as quaternion   
   //current_yaw = tf::getYaw(current_orientation.getRotation()); //calculate yaw in radyans from quaternion orientation

   //**Transfer the Quaternion to Drone current yaw calculated **according to the y axis** in degrees. 
   if(current_yaw<0)current_yaw=3.14159+(current_yaw+3.14159);
   current_yaw=current_yaw*RAD2DEG;
   if(current_yaw<90)current_yaw=90-current_yaw;
   else current_yaw=450-current_yaw;
   //ROS_INFO("Yaw: [%f]",current_yaw);
}

//callback function triggered when a go2point service is received
 bool go2pointService_callback(my_controllers::go2point::Request  &req,
          my_controllers::go2point::Response &res)
 {
   res.received = true;
   ROS_INFO("New goal Service received by uav number %d: x=%f y=%f z=%f",uav_id ,req.goal_x ,req.goal_y ,req.goal_z);
   desired_Pos.x = req.goal_x;   //get desired x Position
   desired_Pos.y = req.goal_y;   //get desired y Position
   desired_Pos.z = req.goal_z;   //get desired z Position
   b=true;                   //enable the flag that there is new goal buplished
   goal_re_srv=false;
   return true;
 }

 bool controllerService_callback(my_controllers::controller_state_srv::Request  &req,
          my_controllers::controller_state_srv::Response &res)
 {
   res.received = true;
   if(req.controllerState) 
   {
        ROS_INFO("The controller has turned ON");
   }
   else
   {
       ROS_INFO("The controller has turned OFF");
   }
   control_is_on = req.controllerState; 
   return true;
 }

//Callback function will be triggered each time something published to /go_to topic
void go_to_Callback(const geometry_msgs::Point::ConstPtr& msg){
  desired_Pos.x = msg->x;   //get desired x Position
  desired_Pos.y = msg->y;   //get desired y Position
  desired_Pos.z = msg->z;   //get desired z Position
  ROS_INFO("New goal topic received by uav number %d: x=%f y=%f z=%f",uav_id,desired_Pos.x ,desired_Pos.y ,desired_Pos.z);
  b=true;                   //enable the flag that there is new goal buplished
  goal_re=false;
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

    //get Parameters from the launch file
    ros::NodeHandle p("~"); //for private parameters
  
  //Parsing the private parameters passed on to the node:
  p.param("uav_id", uav_id, 0); //if you don't pass the "robot_id" argument, it will default as "1".
  std::stringstream id_stream;
  id_stream << uav_id;
  //std::string station_param_x = "/uav" + id_stream.str() + "_station_x";
  //p.param("station_x", station_x, 0.0);
  //std::string station_param_y = "/uav" + id_stream.str() + "_station_y";
  //p.param("station_y", station_y, 0.0);
  //std::string station_param_z = "/uav" + id_stream.str() + "_station_z";
  //p.param("station_z", station_z, 2.0);
  
  //Structure with the Graph Info:
  vertex_set *graph;
  graph = new vertex_set [GRAPH_DIMENSION];
  load_graph(graph);


  std::string sonar_topic = "/uav" + id_stream.str() + "/sonar_height";
  std::string go_to_topic = "/uav" + id_stream.str() + "/go_to";
  std::string ground_topic = "/uav" + id_stream.str() + "/ground_truth/state";
  std::string cmd_topic = "/uav" + id_stream.str() + "/cmd_vel";
  std::string goal_reached_topic = "/uav" + id_stream.str() + "/goal_reached";
    //


  //initializition of services
  std::string go2point_service_name = "/uav" + id_stream.str() + "/go2point";
  ros::ServiceServer service = n.advertiseService(go2point_service_name, go2pointService_callback);
  std::string controller_service_name = "/uav" + id_stream.str() + "/controller_state";
  ros::ServiceServer service1 = n.advertiseService(controller_service_name, controllerService_callback); 
  std::string goal_reached_server = "/uav" + id_stream.str() + "/goalReached"; 
  ros::ServiceClient client = n.serviceClient<my_controllers::goalReached>(goal_reached_server);
  my_controllers::goalReached srv;
  srv.request.goalReached = true;
  //

    //Subscribers and Publishers Declarations
    ros::Subscriber IR_sub = n.subscribe(sonar_topic, 100, &sonar_Callback);    //declaration of /sonar_height topic subscriber
    ros::Subscriber go_to_sub = n.subscribe(go_to_topic, 100, &go_to_Callback);        //declaration of /go_to topic subscriber
    ros::Subscriber Odom_sub = n.subscribe(ground_topic, 100, &Odom_Callback); //declaration of /ground_truth/state topic subscriber
    ros::Publisher Vel_pub = n.advertise<geometry_msgs::Twist>(cmd_topic,100);     //declaration of subscriber to /cmd_vel topic
    ros::Publisher goal_reached = n.advertise<std_msgs::Empty>(goal_reached_topic,100);
    //

    //Structure with the Graph Info:
    //vertex_set *graph;
    //graph = new vertex_set [GRAPH_DIMENSION];
    //load_graph(graph);    //have a look at "graph.cpp" to see what you've just loaded.


    //if there is no desired position received at launch take it as (0,0,0) 
    //desired_Pos.x=graph[start_target_vertex].x;
    //desired_Pos.y=graph[start_target_vertex].y; 
    //desired_Pos.z=graph[start_target_vertex].z; 
    desired_Pos.x=graph[uav_id].x;
    desired_Pos.y=graph[uav_id].y; 
    desired_Pos.z=graph[uav_id].z; 

    /*    
    std::cout<<"Enter the x Position:";
    std::cin>>desired_Pos.x;
    std::cout<<"Enter the Y Position:";
    std::cin>>desired_Pos.y;
    std::cout<<"Enter the Z Position:";
    std::cin>>desired_Pos.z;
    */
ros::Duration(1.0).sleep();
     while (ros::ok()){
if(control_is_on){
        if(b) //if new goal recieved
        {
            b=false;
            //rotate until the drone face the desired new position
    while (((abs(ErrorInOri()))>angle_tolerance)&&(abs(getDistance(current_Pos.x,desired_Pos.x,current_Pos.y,desired_Pos.y,current_Pos.z,desired_Pos.z))>distance_tolerance))
    {
    //vel.linear.x=PidUpdate(0);
    vel.linear.z=PidUpdate(2);
    vel.angular.z=PidUpdate(1);
    vel.linear.x=0;
    vel.linear.y=0;
    vel.angular.x=0;
    vel.angular.y=0;
	Vel_pub.publish(vel);	//publish the new velocity to be applied on the Drone   
    if(control_is_on==0)break;
    ros::spinOnce();	//increase the loop counter by one
	loop_rate.sleep();	//add delay according to the specified loop rate 
     }
        }
    if (getDistance(current_Pos.x,desired_Pos.x,current_Pos.y,desired_Pos.y,current_Pos.z,desired_Pos.z)>distance_tolerance)
     {
         //if the drone facing the desired position move forward while you still controlling the yaw until the drone reaches the goal
    vel.linear.x=PidUpdate(0);
    vel.linear.z=PidUpdate(2);
    vel.angular.z=PidUpdate(1);
	Vel_pub.publish(vel);	//publish the new velocity to be applied on the Drone   
    ros::spinOnce();	//increase the loop counter by one
	loop_rate.sleep();	//add delay according to the specified loop rate 
     }
  
    else
		{
            //if the drone reached the goal stop there
    vel.linear.x=0;
    vel.linear.y=0;
    vel.linear.z=0;	
    vel.angular.x=0;
    vel.angular.y=0;
    vel.angular.z=0;
	Vel_pub.publish(vel);
            if(goal_re_srv==false)
            {
                if(client.call(srv))
                {
                    ROS_INFO("The goal has reached and reported to the patrolling code successfully.");
                    goal_re_srv=true;
                }
            else
                {
                    ROS_INFO("The goal has reached but coudn't be reported to the patrolling code.");
                }
            }
            //send an empty message to the "/uav_id/goal_reached" Topic.
            if(goal_re==false)
            {
                goal_re=true;
            goal_reached.publish(Empty_msg); 
            
            }
    ros::spinOnce();	//increase the loop counter by one
	loop_rate.sleep();	//add delay according to the specified loop rate 
        }
}
else  {
    ros::spinOnce();	//increase the loop counter by one
	loop_rate.sleep();	//add delay according to the specified loop rate 
    }
     }
    delete graph;
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
