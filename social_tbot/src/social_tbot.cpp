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
  Cell c2(2);  
  c1.x = 0;
  c1.y = 100;

  c0.gCost = 0;
  c2.gCost = 10;
  c1.gCost = 100;

  A_star As_obj;

  // Test Cell objects as keys
  As_obj.cameFrom[c0] = c1;
  std::cout << "x:" << As_obj.cameFrom[c0].x << " y:" << As_obj.cameFrom[c0].y << std::endl;

  // Test vector sort
  As_obj.open_set.push_back(c0);   As_obj.open_set.push_back(c1);   As_obj.open_set.push_back(c2);

  for(size_t i = 0; i < As_obj.open_set.size(); i++){
  	std::cout << "ID:" << As_obj.open_set[i].id << " Cost:" << As_obj.open_set[i].gCost << std::endl;
  }

  As_obj.sort_open_set();

  for(size_t i = 0; i < As_obj.open_set.size(); i++){
  	std::cout << "ID:" << As_obj.open_set[i].id << " Cost:" << As_obj.open_set[i].gCost << std::endl;
  }


 return 0;
  
}