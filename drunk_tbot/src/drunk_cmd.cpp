#include "drunk_cmd.h"

TurtleCmdNode::TurtleCmdNode(): kp(10), ki(1), error_cum(0) {
  states.push_back("idle");
  states.push_back("turn_to_goal");
  states.push_back("move_straight");
} //Initialize gains and states 

TurtleCmdNode::~TurtleCmdNode(){} // standard destructor

geometry_msgs::Twist TurtleCmdNode::get_cmd(const turtlesim::PoseConstPtr& msg){
  geometry_msgs::Twist vel_msg;
//  vel_msg.linear.y = 0; vel_msg.linear.z =0; vel_msg.angular.x = 0; vel_msg.angular.y = 0;
  vel_msg.linear.x = -(double)(rand() % 10 +1)/4.0;
  vel_msg.angular.z = 1.0;  
  ROS_INFO("linear x: %f, angular z: %f", vel_msg.linear.x, vel_msg.angular.z);
  return vel_msg;
}


/*void TurtleCmdNode::task_manager*/


void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  //ROS_INFO("I heard: x [%f]", msg->x);
  change_pen(0, 255, 0, 0.01, 0);
  cmd_pub.publish( get_cmd(msg) );

  ROS_INFO("current_state: %s", states[0].c_str());

}

/*
// States:
//    In Motion
//      move forward
//      turn x degrees
//    Idle

// goals:
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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "drunk_cmd");
  TurtleCmdNode turtle;
  turtle.pose_sub = turtle.node.subscribe<turtlesim::Pose>("/turtle1/pose", 1000, boost::bind(&TurtleCmdNode::cmd_callback, &turtle, _1) ); //boost::bind(function, this, placeholder, additional args)
  turtle.cmd_pub = turtle.node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);  
  turtle.pencolor_client = turtle.node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
  ros::spin();
}