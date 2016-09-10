#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"

class TurtleCmdNode{
public:
  ros::NodeHandle node;
  ros::Subscriber pose_sub;
  ros::Publisher  cmd_pub;   

  void cmd_callback(const turtlesim::PoseConstPtr& msg);
  geometry_msgs::Twist get_cmd();

  TurtleCmdNode();
  ~TurtleCmdNode();  
};

TurtleCmdNode::TurtleCmdNode(){} // empty constructor
TurtleCmdNode::~TurtleCmdNode(){} // standard destructor


geometry_msgs::Twist TurtleCmdNode::get_cmd(){
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0.1; //(double)(rand() % 10 +1)/4.0;
  vel_msg.linear.y = 0.1;
  vel_msg.linear.z =0;
  //set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0.1;  
  return vel_msg;

}


void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  ROS_INFO("I heard: x [%f]", msg->x);
  cmd_pub.publish( get_cmd() );
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "drunk_cmd");
  TurtleCmdNode turtle;
  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) );
//  turtle.pose_sub = turtle.node.subscribe("/turtle1/pose", 1000, TurtleCmdNode::cmd_callback);  
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  

  ros::spin();
}