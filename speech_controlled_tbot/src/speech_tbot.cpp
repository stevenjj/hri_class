#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "hlpr_speech_msgs/StampedString.h"

#define FWD_VEL 0.5
#define TURN_VEL 0.5

class TurtleNode{
public:
  ros::NodeHandle node;
  ros::Subscriber speech_sub;
  ros::Publisher  cmd_pub;

  double forward_vel;
  double turn_vel;  

  void speech_callback(const hlpr_speech_msgs::StampedStringConstPtr &msg);

  TurtleNode();
  ~TurtleNode();
};
TurtleNode::TurtleNode(){
	forward_vel = FWD_VEL;
	turn_vel = TURN_VEL;	
}
TurtleNode::~TurtleNode(){}

void TurtleNode::speech_callback(const hlpr_speech_msgs::StampedStringConstPtr &msg){
	geometry_msgs::Twist vel_msg;

	std::string forward_command("GO STRAIGHT");
	std::string turn_left_command("LEFT");
	std::string turn_right_command("RIGHT");		

	if (msg->keyphrase.compare(forward_command) == 0){
		ROS_INFO("The command is to move forward");
		vel_msg.linear.x = forward_vel;
		cmd_pub.publish(vel_msg);
	}
	if (msg->keyphrase.compare(turn_left_command) == 0){
		ROS_INFO("The command is to turn left");
		vel_msg.angular.z = turn_vel;
		cmd_pub.publish(vel_msg);		
	}
	if (msg->keyphrase.compare(turn_right_command) == 0){
		ROS_INFO("The command is to turn right");
		vel_msg.angular.z = -turn_vel;
		cmd_pub.publish(vel_msg);		
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speech_tbot");
  
  TurtleNode turtle;
  turtle.speech_sub = turtle.node.subscribe<hlpr_speech_msgs::StampedString>("/hlpr_speech_commands", 1, boost::bind(&TurtleNode::speech_callback, &turtle, _1));
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  

  ros::spin();

  return 0;
}
