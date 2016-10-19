#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "hlpr_speech_msgs/StampedString.h"

void callback(const hlpr_speech_msgs::StampedStringConstPtr &msg){
	//std::cout << msg->keyphrase << std::endl;

	std::string forward_command("FORWARD");
	std::string turn_left_command("LEFT");
	std::string turn_right_command("RIGHT");		

	if (msg->keyphrase.compare(forward_command) == 0){
		ROS_INFO("The command is to move forward");
	}
	if (msg->keyphrase.compare(turn_left_command) == 0){
		ROS_INFO("The command is to turn left");
	}
	if (msg->keyphrase.compare(turn_right_command) == 0){
		ROS_INFO("The command is to turn right");
	}

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "social_tbot");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<hlpr_speech_msgs::StampedString>("/hlpr_speech_commands", 1, callback);

  ros::spin();

  return 0;
}
