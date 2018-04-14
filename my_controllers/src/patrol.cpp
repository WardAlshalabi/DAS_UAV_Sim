#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Range.h>
#include "Graduation_Trakya/graph.h"
#include <ros/time.h>
#include <std_msgs/Empty.h>
#define GRAPH_DIMENSION 12
#define num_of_uav 3


//Global variables to access within the callbacks:
int uav_id;
//ros::Publisher *comm_pub_ptr;
bool send_goal = false;
bool example_publish = false;
int uav_targets[]={0,0,0}; //initial vertex of drones
int visit_counter[GRAPH_DIMENSION]={0}; //for each vertex count how many times has been visited while recieving communication
float d; //the bigger number in the decision array
float time_vertex[GRAPH_DIMENSION]={0}; //time at each vertex since last visiting
int callback_counter=0;
//Callback triggered everytime this robot reaches a goal (or aborts its goals):
//void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){
//	
//	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
// 		ROS_INFO("Robot %d successfully reached goal.", robot_id);
//		send_goal = true; //go inside the loop again
//		example_publish = true;
//	}else{
// 		ROS_INFO("Robot %d could not reach goal. Gave up!", robot_id);
//	}}


//Callback triggered everytime this robot reaches a goal (or aborts its goals):
    void goal_reached_Callback(const std_msgs::Empty::ConstPtr& msg){
  //ROS_INFO("Drone %d successfully reached goal.", uav_id);
		send_goal = true; //go inside the loop again
		example_publish = true;
        callback_counter++;
        if(callback_counter>1000){callback_counter=2;}
}


//Callback triggered everytime that this robot's goal becomes active:
//void goalActiveCallback(){	
//    ROS_INFO("Drone %d goal is active.", uav_id);
//}


//Callback triggered continuously when a goal is given to the robot:
//void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){ 
  //  ROS_INFO("Robot %d is moving towards a goal.", robot_id);
//}


//Callback triggered everytime you receive a msg from another robot:
void ReceiveCommunicationsCB(const std_msgs::Int16MultiArray::ConstPtr& msg) {

    //Remeber that the format of this message is an array of integers (Int16MultiArray):
    int other_uav_id = msg->data[0];
    int other_uav_vertex = msg->data[1];
	uav_targets[msg->data[0]]=msg->data[1];
	visit_counter[msg->data[1]]++; //count that a specific vertex has been visited for once
        time_vertex[msg->data[1]]=ros::Time::now().toSec();//update the time of vertex
    //filter out your own messages:
    if(uav_id != other_uav_id){ 
       // ROS_INFO("Drone %d received this message from Drone %d: [%d,%d]", uav_id, other_uav_id, msg->data[0], msg->data[1]);
	
    }
}


//A utility function for publishing a message in the "/communication" topic
//void publish_message(int vertex){
    
    //In this example we will publish an array of 2 integers like this: [robot_id, vertex]
    //feel free to modify this function to suit your needs (e.g. publish more information, etc.)
    
 //   std_msgs::Int16MultiArray msg;
 //   msg.data.clear();
 //   msg.data.push_back(uav_id);
 //   msg.data.push_back(vertex);
 //   comm_pub_ptr->publish(msg);
 //   ROS_INFO("Drone %d published: [%d,%d]", uav_id, msg.data[0], msg.data[1]);
//}



int main(int argc, char** argv){ 
    
  // ROS Setup:
  ros::init(argc, argv, "patrol");
  ros::NodeHandle n;
  ros::NodeHandle p("~"); //for private parameters
  
  //Parsing the private parameters passed on to the node:
  p.param("uav_id", uav_id, 0); //if you don't pass the "robot_id" argument, it will default as "0".
  
  std::stringstream id_stream;
  id_stream << uav_id;
  std::string goal_reached_topic = "/uav" + id_stream.str() + "/goal_reached";
  std::string go_to_topic = "/uav" + id_stream.str() + "/go_to";
 // for(int number_of_uav=0;number_of_uav<num_of_uav;number_of_uav++)
//  {
//  id_stream << number_of_uav;
//  std::string first_target_vertex = "uav" + id_stream.str() + "_first_target_vertex";
  p.param("uav0_first_target_vertex", uav_targets[0], 0);
  p.param("uav1_first_target_vertex", uav_targets[1], 1);
  p.param("uav2_first_target_vertex", uav_targets[2], 2);
 // }
  
  //MoveBaseClient ac(move_base_client.c_str(), true);

  for (int t=0;t<GRAPH_DIMENSION;t++) //time initialization
  {
  time_vertex[t]=ros::Time::now().toSec();
  }

  //wait for the action server to become ready:
  //while(!ac.waitForServer(ros::Duration(5.0))){
  //  ROS_INFO("Waiting for the move_base action server to come up");
  //}
  
  //A small sleep of 2.0 seconds for synchronization:
  //ros::Duration(2.2).sleep();

  //Define a publisher to the "/communication" topic as an array of integers (to send data to other robots):
  ros::Publisher go_to_pub = n.advertise<geometry_msgs::Point>(go_to_topic,100);
  ros::Publisher comm_pub = n.advertise<std_msgs::Int16MultiArray>("/communication", 100);
  //comm_pub_ptr = &comm_pub;
  
  //Define a "/communication" subscriber (to receive data from other robots) in the "/communication" topic:
  ros::Subscriber comm_sub = n.subscribe<std_msgs::Int16MultiArray>("/communication", 100, ReceiveCommunicationsCB ); 
  ros::Subscriber goal_reached_sub = n.subscribe<std_msgs::Empty>(goal_reached_topic, 100, goal_reached_Callback ); 
  

  //Navigation goal object:
  geometry_msgs::Point goal;
  
  //Structure with the Graph Info:
  vertex_set *graph;
  graph = new vertex_set [GRAPH_DIMENSION];
  load_graph(graph);    //have a look at "graph.cpp" to see what you've just loaded.

  //x,y targets
  double target_x;                  //in m
  double target_y;                  //in m
  double target_z;
  double target_angle_yaw = 0.0;    //in rads
  int target_vertex=uav_targets[uav_id];
  float D[GRAPH_DIMENSION]={}; //decision array
  float L[GRAPH_DIMENSION]={}; //neighbor array
  bool notzero;
  time_vertex[uav_targets[uav_id]]=ros::Time::now().toSec();
  int loop_counter=0;
  //control variables:

  ros::Time GoalTime;
  
  ros::Rate loop_rate(50); //explicitly define while loop_rate at 50Hz (this will drastically decrease your CPU load)

  while(ros::ok()){ 
    loop_counter++;
    if(loop_counter>1000){loop_counter=3;}
    if (send_goal&&callback_counter>1){
	notzero=false;
	for (int i1=0;i1<GRAPH_DIMENSION;i1++) //update the visits for each vertex
	{
	graph[i1].visits=graph[i1].visits+visit_counter[i1];
	}
	for (int i2=0;i2<GRAPH_DIMENSION;i2++) //reset the visit counter
	{
	visit_counter[i2]=0;
	}
	for (int i3=0;i3<GRAPH_DIMENSION;i3++) //Update time differences
	{
	D[i3]=(float)(ros::Time::now().toSec())-(float)time_vertex[i3];
	}
	for (int i4=0;i4<num_of_uav;i4++) // make the places of robots (the same robot's old target "where it is now" is included) targets in the D array zero.
	{
	D[uav_targets[i4]]=0.0;
	//ROS_INFO("vertex(zero) time is %f.", D[i4]);
    }
	for (int i5=0;i5<GRAPH_DIMENSION;i5++) //reset the neighbor array
	{
	L[i5]=0;
	}
	for (int i6=0;i6<graph[uav_targets[uav_id]].num_neigh;i6++) //1 for neighbor vertex and 0 for non-neighbor
	{
	L[graph[uav_targets[uav_id]].id_neigh[i6]]=1.0;
	}
	for (int i7=0;i7<GRAPH_DIMENSION;i7++) //calculate the decision array
	{
	D[i7]=(float)D[i7]*(float)L[i7];
	}
        for (int z1=0;z1<GRAPH_DIMENSION;z1++) //test that there is at least one non-zero value
	{
	if(D[z1]!=0){notzero=true;break;}
	}
	if(notzero){
	d=D[0]; 		//find the index of the bigger number
	for (int i8=0;i8<GRAPH_DIMENSION;i8++) 
	{
	if(D[i8]>d){d=D[i8];}	       
	}
	for (int k=0;k<GRAPH_DIMENSION;k++) //find the index of the smallest non-zero number (if all the numbers are zeros so target will stay the same and the robot will stay in it's same place)
	{
	if(D[k]==d){target_vertex=k;break;}	       
	}
	}
	
        send_goal = false;  //only sends goal once.
    }
    if(loop_counter==1)
        {
        target_x = graph[target_vertex].x; 
	    target_y = graph[target_vertex].y;   
        target_z = graph[target_vertex].z; 
        goal.x=target_x;
        goal.y=target_y;
        goal.z=target_z;
        
       
        go_to_pub.publish(goal);
        ROS_INFO("Sending goal to uav %d.", uav_id);

        std_msgs::Int16MultiArray msg;
        msg.data.clear();
        msg.data.push_back(uav_id);
        msg.data.push_back(target_vertex);
        comm_pub.publish(msg);
        ROS_INFO("Drone %d published: [%d,%d]", uav_id, msg.data[0], msg.data[1]);
        }
    //publishing a message in the "/communication" topic 10 seconds after sending a goal
    if( example_publish) //&& ros::Time::now() > GoalTime + ros::Duration(10.0) )
    {
      
        target_x = graph[target_vertex].x; 
	target_y = graph[target_vertex].y;   
    target_z = graph[target_vertex].z; 

//ROS_INFO("Target_y is %f.", target_y);
//ROS_INFO("Target_z is %f.", target_z);
        //Example of sending a goal:
        //goal.target_pose.header.frame_id = "map";
        //GoalTime = ros::Time::now();
        //goal.target_pose.header.stamp = GoalTime;
        //geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(target_angle_yaw); //A quaternion must be created to send a goal.
        //goal.target_pose.pose.position.x = target_x;
        //goal.target_pose.pose.position.y = target_y;
        //goal.target_pose.pose.orientation = angle_quat;
        goal.x=target_x;
        goal.y=target_y;
        goal.z=target_z;
        
        //ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
        go_to_pub.publish(goal);
        ROS_INFO("Sending goal to uav %d.", uav_id);

    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(uav_id);
    msg.data.push_back(target_vertex);
    comm_pub.publish(msg);
    ROS_INFO("Drone %d published: [%d,%d]", uav_id, msg.data[0], msg.data[1]);
        example_publish = false; //only publishes once.
        send_goal=false;
    }

    ros::spinOnce(); //needed to make sure that all your callbacks are triggered.
    loop_rate.sleep();
  }
  

  delete graph;
  return 0;
}
