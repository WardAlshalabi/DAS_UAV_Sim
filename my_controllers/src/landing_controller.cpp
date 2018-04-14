#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros_qr_tracker/Percept.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include "Graduation_Trakya/graph.h"
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include "my_controllers/go2point.h"
#include "my_controllers/goalReached.h"
#include "my_controllers/start_landing.h"
#include "my_controllers/controller_state_srv.h"
#include <cstdlib>
#define GRAPH_DIMENSION 15 //remember to edit this number of you change the arena and the number of points
#define num_of_uav 3
#define Battery_Low_Treshold 25
#define Battery_High_Treshold 90
#define HEIGHT_THRESHOLD 0.3 //when the altitude less than this value stop the motors
#define HEIGHT_THRESHOLD2 1 //when the altitude less than this value decrease altitude slowly without QR reading
#define Error_Threshold 30  //when the error between center of image and center of qr code less than this value start decreasing the altitude

int uav_id;
bool battery_low=false;
bool landing_signal_sent=false;
bool uav_in_station=false;
double uav_height;
bool qr_code_detected=false;
int qr_code_center[2]={0,0};
int image_center[2]={0,0};
geometry_msgs::Twist vel;
double rate=50;                     //Loop rate in HZ
double period;
double p=0.005;
double i=0.00;
double d=0.0001;
double intE_max[2]={5,5};
double proE[2],proEP[2],derE[2],intE[2];
double PidUpdate(int a);


 bool start_landing_callback(my_controllers::start_landing::Request  &req,
          my_controllers::start_landing::Response &res)
 {
    landing_signal_sent=true;
    res.received = true;
    ROS_INFO("UAV with ID number %d started landing.",uav_id);
    return true;
 }

void battery_callback(const sensor_msgs::BatteryState::ConstPtr& msg) {

    //Remeber that the format of this message is an array of integers (Int16MultiArray):
    int Charge = msg->charge;
    if(battery_low)
    {
        if (Charge>Battery_High_Treshold) battery_low=false; 
    }
    else
    {
        if(Charge<Battery_Low_Treshold) battery_low=true;
    }
    }

void qr_code_Callback(const ros_qr_tracker::Percept::ConstPtr& msg) {
    std::stringstream id_stream;
    id_stream << uav_id;
    std::string Station_name = "Drone_Station_" + id_stream.str();
    qr_code_center[0]=msg->center_x;
    qr_code_center[1]=msg->center_y;
    image_center[0]=msg->width/2;
    image_center[1]=msg->height/2;
    //ROS_INFO("Data:%s.",msg->data.c_str());
    if(msg->data==Station_name)
    {
        qr_code_detected=true;
        //ROS_INFO("The Station of UAV%d has been detected.",uav_id);
        //ROS_INFO("Coordinate of station center: x=%d, y=%d",msg->center_x,msg->center_y);
    }

}

void sonar_Callback(const sensor_msgs::Range::ConstPtr& msg){
  uav_height = msg->range;
}

    int main(int argc, char** argv){ 
    
  // ROS Setup:
  ros::init(argc, argv, "landing_controller");
  ros::NodeHandle n;
  ros::NodeHandle p("~"); //for private parameters
  
  //Parsing the private parameters passed on to the node:
  p.param("uav_id", uav_id, 0); //if you don't pass the "robot_id" argument, it will default as "0".
  
  std::stringstream id_stream;
  id_stream << uav_id;
  std::string Battery_topic_name = "/uav" + id_stream.str() + "/Battery_state";
  ros::Subscriber battery_sub = n.subscribe<sensor_msgs::BatteryState>(Battery_topic_name, 100, battery_callback ); 
  std::string sonar_topic = "/uav" + id_stream.str() + "/sonar_height";
  ros::Subscriber IR_sub = n.subscribe(sonar_topic, 100, &sonar_Callback);
  std::string qr_code_topic = "/qr_tracker_" + id_stream.str() + "/matches";
  ros::Subscriber QR_sub = n.subscribe(qr_code_topic, 100, &qr_code_Callback);

  std::string start_landing_server = "/uav" + id_stream.str() + "/start_landing"; 
  ros::ServiceServer service = n.advertiseService(start_landing_server, start_landing_callback); 
  std::string controller_state_service_name = "/uav" + id_stream.str() + "/controller_state";
  ros::ServiceClient client = n.serviceClient<my_controllers::controller_state_srv>(controller_state_service_name);
  my_controllers::controller_state_srv srv;
  std::string cmd_topic = "/uav" + id_stream.str() + "/cmd_vel";
  ros::Publisher Vel_pub = n.advertise<geometry_msgs::Twist>(cmd_topic,100);

  ros::Rate loop_rate(rate);
  period = 1/rate;
  while(ros::ok()){ 
      if(landing_signal_sent)
      {
        landing_signal_sent=false;
        uav_in_station=true;
        srv.request.controllerState=false;
        if (client.call(srv))
        {
            ROS_INFO("Controller of UAV with ID number %d has been turnedoff.",uav_id);
        }
        else
        {
            ROS_ERROR("could not turnoff the controller of UAV number %d.",uav_id);
        }
      }
      if(uav_height>HEIGHT_THRESHOLD&&uav_in_station&&qr_code_detected)
      {
          //write program for centering the uav with QRcode
        qr_code_detected=false;
        vel.linear.x=PidUpdate(0);
        vel.linear.y=PidUpdate(1);
        if(abs(image_center[1]-qr_code_center[1])<Error_Threshold&&abs(image_center[0]-qr_code_center[0])<Error_Threshold)
        {vel.linear.z=-0.5;}
        else vel.linear.z=0;
        vel.angular.x=0;
        vel.angular.y=0;
        vel.angular.z=0;
	    Vel_pub.publish(vel);
      }
      else if(uav_height>HEIGHT_THRESHOLD&&uav_in_station&&qr_code_detected==false)
      {
          //write program for centering the uav with QRcode
        //vel.linear.x=0.5;
        vel.linear.x=0;
        vel.linear.y=0;
        vel.linear.z=0;
        vel.angular.x=0;
        vel.angular.y=0;
        //vel.angular.z=1;
        vel.angular.z=0;
	    Vel_pub.publish(vel);
      }
      if(uav_height<HEIGHT_THRESHOLD2&&uav_height>HEIGHT_THRESHOLD&&uav_in_station)
      {
          vel.linear.x=0;
        vel.linear.y=0;
        vel.linear.z=-0.5;
        vel.angular.x=0;
        vel.angular.y=0;
        vel.angular.z=0;
	    Vel_pub.publish(vel);
      }
      if(battery_low==false&&uav_in_station)
      {
        uav_in_station=false;
        srv.request.controllerState=true;
        if (client.call(srv))
        {
            ROS_INFO("The UAV with ID number %d is fully charged and ready to takeoff.",uav_id);
        }
        else
        {
            ROS_ERROR("could not rerun the controller of UAV number %d.",uav_id);
        }
      }
      ros::spinOnce(); //needed to make sure that all your callbacks are triggered.
      loop_rate.sleep();
  }
  return 0;
    }


    double PidUpdate(int a)
{
proEP[a]=proE[a];	//save the last propotional error as pervious propotional error
//calculate the new propotional error
switch(a) 
{
case 0: //The PID value will be calculated for Linear velocity
    proE[a]=image_center[1]-qr_code_center[1];	
    break;

case 1: //The PID value will be calculated for Angular velocity
    proE[a]=image_center[0]-qr_code_center[0];
    break;
}
//
derE[a]=proE[a]-proEP[a];		//calculate the derivative error
intE[a]=intE[a]+proE[a];			//calculate the integer error
if(intE[a]>intE_max[a])intE[a]=intE_max[a];	//limiting for the integer error
return (p*proE[a]) + (i*intE[a]*(period)) + (d*(derE[a]/(period)));	//calculate the new PID function's output value
}