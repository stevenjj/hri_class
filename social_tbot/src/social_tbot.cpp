#include "social_tbot.h"

int main(int argc, char **argv)
{
/*  ros::init(argc, argv, "drunk_cmd");
  TurtleCmdNode turtle;
  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  
  turtle.pencolor_client = turtle.node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::spin();*/

  int resolution = 100; // Number of cells per dimension
  float x_min = 0; float y_min = 0; float x_max = 10; float y_max = 10;


  A_star As_obj(resolution, x_min, x_max, y_min, y_max);
  std::cout << As_obj.grid.size() << std::endl;


  float num = 9.92;
  std::cout << As_obj.x_location_to_map_index(num) << std::endl;
  std::cout << As_obj.y_location_to_map_index(num) << std::endl;  

  float x_try = 9.95;
  float y_try = 9.95;  
  std::cout << As_obj.x_location_to_map_index(x_try) << "," << As_obj.y_location_to_map_index(y_try) << std::endl;  
  std::cout << As_obj.convert_xy_to_cell(x_try, y_try).grid_x_loc << "," << As_obj.convert_xy_to_cell(x_try, y_try).grid_y_loc << std::endl;

  std::vector<Cell> list_of_neighbors = As_obj.neighbors(As_obj.convert_xy_to_cell(x_try, y_try));

  std::cout << "Neighbors of (" << x_try << "," << y_try << ")" << std::endl;
  for(size_t i = 0; i < list_of_neighbors.size(); i++){
  	std::cout << "    gridX:" << list_of_neighbors[i].grid_x_loc << " gridY:" << list_of_neighbors[i].grid_y_loc << " id:" << list_of_neighbors[i].id << std::endl;
  }

/*  
  Cell c0(0);
  Cell c1(1);
  Cell c2(2);  
  c1.x = 0;
  c1.y = 100;

  c0.gCost = 0;
  c2.gCost = 10;
  c1.gCost = 100;

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
  }*/



 return 0;
  
}