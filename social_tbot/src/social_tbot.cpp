#include "social_tbot.h"
#include "social_cmd.h"
#include <Eigen/Dense>


void TurtleCmdNode::update_current_pose(const turtlesim::PoseConstPtr& msg){
  current_pose = *msg;
    //ROS_INFO("current: x [%f]", msg->x);
}
void TurtleCmdNode::update_state(int new_state){
  current_state = new_state;
}

void TurtleCmdNode::do_next_task(){
  current_task += 1;
}

void TurtleCmdNode::change_pen(int r, int g, int b, int width, int off){
  pen_srv.request.r = r;
  pen_srv.request.g = g;
  pen_srv.request.b = b;
  pen_srv.request.width = width;
  pen_srv.request.off = off;

  if (pencolor_client.call(pen_srv))
  {
    ROS_INFO("Color Change");
  }
}

void TurtleCmdNode::set_goal(float goal_x, float goal_y){
  goal_pose.x = goal_x;
  goal_pose.y = goal_y;  

//  goal_pose.theta = deg_2_rad(goal_theta_deg);
  goal_pose.theta = atan2( (goal_y - current_pose.y), 
                           (goal_x - current_pose.x) );

  update_state(STATE_TURN_TO_GOAL);

}

float TurtleCmdNode::calculate_linear_error(){
      // Calculate errors
      float error_x = (goal_pose.x - current_pose.x);
      float error_y = (goal_pose.y - current_pose.y);
      float error_mag = sqrt( pow(error_x,2) + pow(error_y, 2) );

      float goal_dir_x = cos(goal_pose.theta);
      float goal_dir_y = sin(goal_pose.theta);  
      
      // Dot product of heading and e_pos = [dx, dy]^T
      float error_sign = sign((goal_dir_x*error_x) + (goal_dir_y*error_y)); 
      return error_mag*error_sign;
}

float TurtleCmdNode::calculate_heading_error(){

}

geometry_msgs::Twist TurtleCmdNode::get_cmd(const turtlesim::PoseConstPtr& msg){
  geometry_msgs::Twist vel_msg;
//  vel_msg.linear.y = 0; vel_msg.linear.z =0; vel_msg.angular.x = 0; vel_msg.angular.y = 0; 
  float linear_command = 0;
  float heading_command = 0;  

  //ROS_INFO("STATE %i", current_state);
  // Calculate Command
  // STATE: Turn to Goal
  if (current_state == STATE_TURN_TO_GOAL){

    float heading_error = goal_pose.theta - current_pose.theta;
  	ROS_INFO("goal pose: %f ", goal_pose.theta);
  	ROS_INFO("current_pose: %f ", current_pose.theta);  	 
    ROS_INFO("heading_error: %f", heading_error);
    if (fabs(heading_error) < 0.01){
      heading_command = 0;
      if (drunk_motion == true){
        update_state(STATE_MOVE_TO_GOAL);
      }else{
        update_state(STATE_MOVE_STRAIGHT);      
      }
    }else{
      heading_command = kp_head*heading_error;
    }

  // STATE: MOVE STRAIGHT
  }else if(current_state == STATE_MOVE_STRAIGHT){
      float linear_error = calculate_linear_error();
      error_cum += linear_error;      
      // Calculate Control Command
      linear_command = kp_lin*linear_error - kd_lin*current_pose.linear_velocity + ki_lin*error_cum;

      //ROS_INFO("mag: %f", linear_error);
      if (fabs(linear_error) < 0.01){
      //if (fabs(linear_error) < 0.02){
        error_cum = 0;
        internal_time = 0;
        update_state(STATE_IDLE);
      }

  // STATE: MOVE TO GOAL
  }else if(current_state == STATE_MOVE_TO_GOAL){
      
      float heading_error = goal_pose.theta - current_pose.theta;
      float linear_error = calculate_linear_error();

      //ROS_INFO("heading error: %f", heading_error);

      if (start_motion == false){
        start_motion = true;
        init_error_dist = linear_error;
      }

      error_cum += linear_error;      
      // Calculate Control Command
      linear_command = kp_lin*linear_error;// - kd_lin*current_pose.linear_velocity + ki_lin*error_cum;      
      heading_command = kp_head*heading_error;// + (double)(rand() % 10 +1)/4.0;

      float b = 5; float a = -5;
      //float random_num = 1 + (b-a) * ((double)(rand() % 100) / 100.0) + a;
      //heading_command += ( (10+random_num)  *cos(25*internal_time));

//      heading_command += ( 0.5*sin(100*PI/init_error_dist)*linear_error);
      internal_time += internal_dt;     

      //ROS_INFO("mag: %f", linear_error);
      if (fabs(linear_error) < 0.25){
        internal_time = 0;
        error_cum = 0;
        update_state(STATE_IDLE);
      }
  }
  // STATE: IDLE    
  else{
    ROS_INFO("IDLE");
    linear_command = 0;
    heading_command = 0;    
  }

  // Finalize command
  vel_msg.linear.x = max_linear_command(linear_command)*sign(linear_command);//(double)(rand() % 10 +1)/4.0;
  vel_msg.angular.z = max_heading_command(heading_command)*sign(heading_command); 
  //ROS_INFO("linear x: %f, angular z: %f", vel_msg.linear.x, vel_msg.angular.z);
  return vel_msg;
}

void TurtleCmdNode::pose_callback(const turtlesim::PoseConstPtr& msg){
	turtlesim::Pose turtle2_pose;
	turtle2_pose = *msg;
	As_obj.turtle_2x = turtle2_pose.x;
	As_obj.turtle_2y = turtle2_pose.y;	
	turtle2_loc = true;

	//std::cout << "turtle 2:" << As_obj.turtle_2x << std::endl;

}

void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  // Update state here
  update_current_pose(msg);

  float x_goal = 9.0;
  float y_goal = 9.0;  

  if (turtle2_loc == true){

	  if (compute_once == false){
		path = As_obj.find_path(current_pose.x, current_pose.y, x_goal, y_goal);
		std::cout << current_pose.x << std::endl;
		std::cout << current_pose.y << std::endl;  
	  	std::cout << "    gridX:" << path[0].grid_x_loc << " gridY:" << path[0].grid_y_loc << std::endl;	
	  	std::cout << "    gridX:" << path[1].grid_x_loc << " gridY:" << path[1].grid_y_loc << std::endl;	
	    std::cout << "Length of path " << path.size() << std::endl;
	    compute_once = true;
	  }

	  // Main State Machine Logic
	  if ((current_state == STATE_TURN_TO_GOAL) || (current_state == STATE_MOVE_STRAIGHT) ||
	       (current_state == STATE_MOVE_TO_GOAL)){
	    cmd_pub.publish( get_cmd(msg) );    
	  }else if(current_state == STATE_IDLE){


	    do_next_task();  
	    update_task();  

	  }
	}

}



void TurtleCmdNode::update_task(){
  // Orange: R: 255 G:162  B:0
    change_pen(255, 162, 0, 5, 0);
    if (current_task < path.size()){
	    set_goal(path[current_task].x, path[current_task].y);
		std::cout << "    gridX:" << path[current_task].grid_x_loc << " gridY:" << path[current_task].grid_y_loc << 	
					"cost" << path[current_task].gCost << std::endl;	
	}
  	

/*  if (current_task == 0){
    current_task_description = "Go to next waypoint";
    change_pen(255, 162, 0, 5, 0);
    set_goal(path[current].x, path[1].y);
  // DRAW U
  }else if(current_task == 1){
    current_task = -1;
  }else{
    ROS_INFO("Finished all tasks");

  }*/

}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "social_tbot");
  TurtleCmdNode turtle;
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

/*
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
*/

/*  float x_start = 5; float y_start = 5; float x_goal = 9; float y_goal = 9;

  std::vector<Cell> final_path = As_obj.find_path(x_start, y_start, x_goal, y_goal);
  std::cout << "Length of path" << final_path.size() << std::endl;

  for(size_t i=0; i < final_path.size() ; i++){
  	std::cout << "    gridX:" << final_path[i].grid_x_loc << " gridY:" << final_path[i].grid_y_loc << std::endl;	
  }*/

  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.turtle2_pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle2/pose", 1000, boost::bind(&TurtleCmdNode::pose_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  
  turtle.pencolor_client = turtle.node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::spin();



 return 0;
  
}