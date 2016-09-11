#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"

#include "turtlesim/SetPen.h"

class TurtleCmdNode{
public:
  ros::NodeHandle node;
  ros::Subscriber pose_sub;
  ros::Publisher  cmd_pub;   
  ros::ServiceClient pencolor_client;

  turtlesim::SetPen pen_srv;

  void cmd_callback(const turtlesim::PoseConstPtr& msg);
  void change_pen_color(int r, int g, int b, int width, int off);

  geometry_msgs::Twist get_cmd(const turtlesim::PoseConstPtr& msg);

  TurtleCmdNode();
  ~TurtleCmdNode();  
};

TurtleCmdNode::TurtleCmdNode(){} // empty constructor
TurtleCmdNode::~TurtleCmdNode(){} // standard destructor


geometry_msgs::Twist TurtleCmdNode::get_cmd(const turtlesim::PoseConstPtr& msg){
  geometry_msgs::Twist vel_msg;
//  vel_msg.linear.y = 0; vel_msg.linear.z =0; vel_msg.angular.x = 0; vel_msg.angular.y = 0;
  vel_msg.linear.x =    -(double)(rand() % 10 +1)/4.0;
  vel_msg.angular.z = 1.0;  
  ROS_INFO("linear x: %f, angular z: %f", vel_msg.linear.x, vel_msg.angular.z);
  return vel_msg;
}

void TurtleCmdNode::change_pen_color(int r, int g, int b, int width, int off){
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

error = (x_goal - x_start)

vel = 



*/
void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  //ROS_INFO("I heard: x [%f]", msg->x);
  change_pen_color(0, 255, 0, 0.01, 0);
  cmd_pub.publish( get_cmd(msg) );
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