#include "social_tbot.h"

int main(int argc, char **argv)
{
/*  ros::init(argc, argv, "drunk_cmd");
  TurtleCmdNode turtle;
  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  
  turtle.pencolor_client = turtle.node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::spin();*/
  std::cout << "hello world" << std::endl;

  Cell c0(0);
  Cell c1(1);
  c1.x = 0;
  c1.y = 100;

  A_star As_obj;

  As_obj.cameFrom[c0] = c1;
  std::cout << "x:" << As_obj.cameFrom[c0].x << " y:" << As_obj.cameFrom[c0].y << std::endl;


 return 0;
  
}