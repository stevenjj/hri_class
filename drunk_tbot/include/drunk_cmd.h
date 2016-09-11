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


class TurtleCmdNode{
public:
  ros::NodeHandle node;
  ros::Subscriber pose_sub;
  ros::Publisher  cmd_pub;   
  ros::ServiceClient pencolor_client;

  turtlesim::SetPen pen_srv;

  // Robot States
  std::vector<int> states;
  turtlesim::Pose current_pose;
  turtlesim::Pose goal_pose;
  int current_state;
  int current_task;

  float kp;
  float ki;    
  float error_cum;


  void cmd_callback(const turtlesim::PoseConstPtr& msg);
  void update_current_pose(const turtlesim::PoseConstPtr& msg);

  void change_pen(int r, int g, int b, int width, int off);

  void set_goal(float goal_x, float goal_y, float goal_theta_deg);
  void update_task();

  geometry_msgs::Twist get_cmd(const turtlesim::PoseConstPtr& msg);


  TurtleCmdNode();
  ~TurtleCmdNode();  
};

TurtleCmdNode::TurtleCmdNode(): kp(1), ki(1), error_cum(0) {
  states.push_back(STATE_IDLE);
  states.push_back(STATE_MOVE_STRAIGHT);
  states.push_back(STATE_TURN_TO_GOAL);
  current_state = STATE_IDLE;
} //Initialize gains and states 

TurtleCmdNode::~TurtleCmdNode(){} // standard destructor

#endif
