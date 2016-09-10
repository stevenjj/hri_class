#include "ros/ros.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"

class TurtleCmdNode{
public:
  ros::NodeHandle node;
  ros::Subscriber pose_sub;
  ros::Publisher  cmd_pub;  

  TurtleCmdNode();
  ~TurtleCmdNode();  
};


void chatterCallback(const turtlesim::Pose& msg)
{
  ROS_INFO("I heard: x [%f]", msg.x);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drunk_cmd");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/turtle1/pose", 1000, chatterCallback);
  ros::spin();
}