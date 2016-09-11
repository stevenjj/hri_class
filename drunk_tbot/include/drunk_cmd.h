#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Color.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"

#include <Eigen/Dense>

class TurtleCmdNode{
public:
  ros::NodeHandle node;
  ros::Subscriber pose_sub;
  ros::Publisher  cmd_pub;   
  ros::ServiceClient pencolor_client;

  turtlesim::SetPen pen_srv;

  // Robot States
  std::vector<std::string> states;
  std::string current_state;
  float kp;
  float ki;    
  float error_cum;


  void cmd_callback(const turtlesim::PoseConstPtr& msg);
  void change_pen(int r, int g, int b, int width, int off);

  geometry_msgs::Twist get_cmd(const turtlesim::PoseConstPtr& msg);

  TurtleCmdNode();
  ~TurtleCmdNode();  
};