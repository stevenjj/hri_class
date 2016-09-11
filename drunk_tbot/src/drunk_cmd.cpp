#include "drunk_cmd.h"

// State machine:
//  turn to goal -> move_straight -> idle

float rad_2_deg(float rads){
  return (180.0/PI)*rads;
}

float deg_2_rad(float degs){
  return (PI/180.0)*degs;
}

void TurtleCmdNode::update_current_pose(const turtlesim::PoseConstPtr& msg){
    current_pose = *msg;
    //ROS_INFO("current: x [%f]", msg->x);
}
void TurtleCmdNode::change_pen(int r, int g, int b, int width, int off){
  pen_srv.request.r = r;
  pen_srv.request.g = g;
  pen_srv.request.b = b;
  pen_srv.request.width = r;
  pen_srv.request.off = off;

  if (pencolor_client.call(pen_srv))
  {
    ROS_INFO("Color Change");
  }
}
void TurtleCmdNode::set_goal(float goal_x, float goal_y, float goal_theta_deg){
  goal_pose.x = goal_x;
  goal_pose.x = goal_y;  
  goal_pose.theta = deg_2_rad(goal_theta_deg);
  current_state = STATE_TURN_TO_GOAL;
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
    if (fabs(heading_error) < 0.05){
      heading_command = 0;
    }else{
      heading_command = kp*heading_error;
    }

  // STATE: MOVE STRAIGHT
  }else if(current_state == STATE_MOVE_STRAIGHT){
    vel_msg.angular.z = 0;

  // STATE: MUST BE IDLE    
  }else{
    linear_command = 0;
    heading_command = 0;    
  }


  // Finalize command
  vel_msg.linear.x = linear_command;//(double)(rand() % 10 +1)/4.0;
  vel_msg.angular.z = heading_command; 
  ROS_INFO("linear x: %f, angular z: %f", vel_msg.linear.x, vel_msg.angular.z);
  return vel_msg;
}


void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  // Update state here
  update_current_pose(msg);


  update_task();

  ROS_INFO("current_state: %i", states[0]);
  cmd_pub.publish( get_cmd(msg) );
}

void TurtleCmdNode::update_task(){
  if (current_task == 0){
    change_pen(0, 255, 0, 0.01, 0);
    set_goal(current_pose.x, current_pose.y, 0);
  }else if(current_task == 1){

  }else if(current_task == 2){

  }else{

  }

}
/*
// States:
//    In Motion
//      move forward
//      turn x degrees
//    Idle

goal_x = ;
goal_y = ;
goal_heading = ;
heading_goal;
if state is in motion = [move or turn]:
  if state is move forward:
    continue moving forward
  elif state is turn heading:
    continue turning
elif state is idle:
  move_to_next_task
  set_new_goals and change goal_x, goal_y, goal_heading
  change pen color

// Tasks:
// Draw U
// Move to starting position
// Draw T

if state == idle {
  move to next goal  
}

x_start = x_o
calculate delta_x:
heading: x_hat = cos(th)*i + sin(th)*j
x_goal = x_hat*number of steps

error = ||x_goal - x_start||_2 *  sign(heading_dir dot ||x_goal-x_start||_2

vel = error*kp



*/




int main(int argc, char **argv)
{
  ros::init(argc, argv, "drunk_cmd");
  TurtleCmdNode turtle;
  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  
  turtle.pencolor_client = turtle.node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::spin();
}