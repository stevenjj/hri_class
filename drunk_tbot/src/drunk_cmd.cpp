#include "drunk_cmd.h"

// State machine:
//  turn to goal -> move_straight -> idle
//  @ Idle update task
//  if task is empty, do nothing.

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

geometry_msgs::Twist TurtleCmdNode::get_cmd(const turtlesim::PoseConstPtr& msg){
  geometry_msgs::Twist vel_msg;
//  vel_msg.linear.y = 0; vel_msg.linear.z =0; vel_msg.angular.x = 0; vel_msg.angular.y = 0; 
  float linear_command = 0;
  float heading_command = 0;  

  // Calculate Command
  // STATE: Turn to Goal
  if (current_state == STATE_TURN_TO_GOAL){
    float heading_error = goal_pose.theta - current_pose.theta;
    ROS_INFO("heading_error: %f", heading_error);
    if (fabs(heading_error) < 0.01){
      heading_command = 0;
      update_state(STATE_MOVE_STRAIGHT);
    }else{
      heading_command = kp_head*heading_error;
    }

  // STATE: MOVE STRAIGHT
  }else if(current_state == STATE_MOVE_STRAIGHT){
      // Calculate errors
      float error_x = (goal_pose.x - current_pose.x);
      float error_y = (goal_pose.y - current_pose.y);
      float error_mag = sqrt( pow(error_x,2) + pow(error_y, 2) );

      float goal_dir_x = cos(goal_pose.theta);
      float goal_dir_y = sin(goal_pose.theta);  
      
      // Dot product of heading and e_pos = [dx, dy]^T
      float error_sign = sign((goal_dir_x*error_x) + (goal_dir_y*error_y)); 
      error_cum += (error_sign*error_mag);      

      // Calculate Control Command
      linear_command = kp_lin*error_sign*error_mag - kd_lin*current_pose.linear_velocity + ki_lin*error_cum;

      //ROS_INFO("goal_x: %f, current_x: %f, goal_y: %f, current_y: %f", goal_pose.x, current_pose.x, goal_pose.y, current_pose.y);
      ROS_INFO("mag: %f", error_mag);

      if (error_mag < 0.12){
        error_cum = 0;
        update_state(STATE_IDLE);
      }

  // STATE: IDLE    
  }else{
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


void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  // Update state here
  update_current_pose(msg);

  // Main State Machine Logic
  if ((current_state == STATE_TURN_TO_GOAL) || (current_state == STATE_MOVE_STRAIGHT)){
    cmd_pub.publish( get_cmd(msg) );    
  }else if(current_state == STATE_IDLE){
    do_next_task();
    update_task();    
  }

}

void TurtleCmdNode::update_task(){
  if (current_task == 0){
    current_task_description = "Go to starting position";
    change_pen(0, 255, 0, 3, 0);
    set_goal(1, 9);
//    set_goal(current_pose.x, current_pose.y, 0);
  }else if(current_task == 1){
    change_pen(255, 0, 0, 3, 0);
    set_goal(9,1);
  }else if(current_task == 2){
    change_pen(0, 0, 255, 3, 0);
    set_goal(9,9);
  }else if(current_task == 3){
    change_pen(0, 255, 0, 3, 0);
    set_goal(1,1);
    current_task = -1;    
  }else{
    ROS_INFO("Finished all tasks");

  }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "drunk_cmd");
  TurtleCmdNode turtle;
  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  
  turtle.pencolor_client = turtle.node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::spin();
}