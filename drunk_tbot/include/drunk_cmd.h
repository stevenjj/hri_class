#ifndef DRUNK_CMD_HEADER
#define DRUNK_CMD_HEADER

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"

#include <Eigen/Dense>

#define PI 3.14159265359

#define STATE_IDLE 0
#define STATE_MOVE_STRAIGHT 1
#define STATE_TURN_TO_GOAL 2

#define STATE_MOVE_TO_GOAL 3


class TurtleCmdNode{
public:
  ros::NodeHandle node;
  ros::Subscriber pose_sub;
  ros::Publisher  cmd_pub;   
  ros::ServiceClient pencolor_client;

  turtlesim::SetPen pen_srv;

  // Robot States
  float internal_time;
  float internal_dt;
  float omega;
  std::vector<int> states;
  turtlesim::Pose current_pose;
  turtlesim::Pose goal_pose;
  int current_state;
  int current_task;

  std::string current_task_description;

  float kp_head;
  float ki_head;  

  float kp_lin;
  float kd_lin;
  float ki_lin;    
  float error_cum;

  bool start_motion;
  float init_error_dist;

  float MAX_LINEAR_VEL;
  float MAX_ANGULAR_VEL;  

  void cmd_callback(const turtlesim::PoseConstPtr& msg);
  void update_current_pose(const turtlesim::PoseConstPtr& msg);
  void update_state(int new_state);
  void do_next_task();  

  void change_pen(int r, int g, int b, int width, int off);

  void set_goal(float goal_x, float goal_y);
  void update_task();

  geometry_msgs::Twist get_cmd(const turtlesim::PoseConstPtr& msg);


  float calculate_linear_error();
  float calculate_heading_error();  

  float max_linear_command(float linear_command);
  float max_heading_command(float heading_command);  


  TurtleCmdNode();
  ~TurtleCmdNode();  
};

TurtleCmdNode::TurtleCmdNode(): kp_head(12), ki_head(0.01), kp_lin(1), kd_lin(0.05), ki_lin(0), error_cum(0),
                                current_task(-1), MAX_LINEAR_VEL(5), MAX_ANGULAR_VEL(5) {
  start_motion = false;
  internal_time = 0.0;
  internal_dt = 0.01;
  omega = 10;
  states.push_back(STATE_IDLE);
  states.push_back(STATE_MOVE_STRAIGHT);
  states.push_back(STATE_TURN_TO_GOAL);
  current_state = STATE_IDLE;

} //Initialize gains and states 

TurtleCmdNode::~TurtleCmdNode(){} // standard destructor

// Simple utils
float rad_2_deg(float rads){
  return (180.0/PI)*rads;
}

float deg_2_rad(float degs){
  return (PI/180.0)*degs;
}

float sign(float num){
  if (num > 0){
    return 1;
  }else if (num < 0){
    return -1;
  }else{
    return 0;
  }
}

float TurtleCmdNode::max_linear_command(float linear_command){
  if (fabs(linear_command) > MAX_LINEAR_VEL){
    return MAX_LINEAR_VEL;
  }else{
    return fabs(linear_command);
  }
}

float TurtleCmdNode::max_heading_command(float heading_command){
  if (fabs(heading_command) > MAX_ANGULAR_VEL){
    return MAX_ANGULAR_VEL;
  }else{
    return fabs(heading_command);
  }
}

#endif
