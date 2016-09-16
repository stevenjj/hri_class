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

  ROS_INFO("STATE %i", current_state);
  // Calculate Command
  // STATE: Turn to Goal
  if (current_state == STATE_TURN_TO_GOAL){
    float heading_error = goal_pose.theta - current_pose.theta;
    ROS_INFO("heading_error: %f", heading_error);
    if (fabs(heading_error) < 0.001){
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

      ROS_INFO("mag: %f", linear_error);
      if (fabs(linear_error) < 0.25){
      //if (fabs(linear_error) < 0.02){
        error_cum = 0;
        internal_time = 0;
        update_state(STATE_IDLE);
      }

  // STATE: MOVE TO GOAL
  }else if(current_state == STATE_MOVE_TO_GOAL){
      
      float heading_error = goal_pose.theta - current_pose.theta;
      float linear_error = calculate_linear_error();

      ROS_INFO("heading error: %f", heading_error);

      if (start_motion == false){
        start_motion = true;
        init_error_dist = linear_error;
      }

      error_cum += linear_error;      
      // Calculate Control Command
      linear_command = kp_lin*linear_error;// - kd_lin*current_pose.linear_velocity + ki_lin*error_cum;      
      heading_command = kp_head*heading_error;// + (double)(rand() % 10 +1)/4.0;

      float b = 5; float a = -5;
      float random_num = 1 + (b-a) * ((double)(rand() % 100) / 100.0) + a;
      heading_command += ( (10+random_num)  *cos(25*internal_time));

//      heading_command += ( 0.5*sin(100*PI/init_error_dist)*linear_error);
      internal_time += internal_dt;     

      ROS_INFO("mag: %f", linear_error);
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


void TurtleCmdNode::cmd_callback(const turtlesim::PoseConstPtr& msg){
  // Update state here
  update_current_pose(msg);

  // Main State Machine Logic
  if ((current_state == STATE_TURN_TO_GOAL) || (current_state == STATE_MOVE_STRAIGHT) ||
       (current_state == STATE_MOVE_TO_GOAL)){
    cmd_pub.publish( get_cmd(msg) );    
  }else if(current_state == STATE_IDLE){
    do_next_task();
    update_task();    
  }

}

void TurtleCmdNode::update_task(){
  // Orange: R: 255 G:162  B:0

  if (current_task == 0){
    current_task_description = "Go to starting position";
    change_pen(255, 162, 0, 5, 1);
    set_goal(init_x, init_y);

  // DRAW U
  }else if(current_task == 1){
    change_pen(255, 162, 0, 5, 0);
    set_goal(current_pose.x, current_pose.y - step_size);
  }else if(current_task == 2){
    set_goal(current_pose.x + step_size, current_pose.y);
  }else if(current_task == 3){
    set_goal(current_pose.x, current_pose.y + step_size); 

  // Go to next letter   
  }else if(current_task == 4){
    change_pen(255, 162, 0, 5, 1);    
    set_goal(current_pose.x + step_size/4, current_pose.y);

  // Draw T
  }else if(current_task == 5){
    change_pen(255, 162, 0, 5, 0);
    set_goal(current_pose.x + step_size, current_pose.y);
  }else if(current_task == 6){
    change_pen(255, 162, 0, 5, 1);        
    set_goal(current_pose.x - step_size/2, current_pose.y);
  }else if(current_task == 7){
    change_pen(255, 162, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y - step_size);

  // Next Letter
  }else if(current_task == 8){
    change_pen(255, 162, 0, 5, 1);        
    set_goal(current_pose.x, current_pose.y - step_size/4);

  // Draw h
  }else if(current_task == 9){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y - step_size);    
  }else if(current_task == 10){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y + step_size/2);
  }else if(current_task == 11){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x + step_size/2, current_pose.y);
  }else if(current_task == 12){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y- step_size/2 );

  // Next Letter  
  }else if(current_task == 13){
    change_pen(0, 0, 0, 5, 1);        
    set_goal(current_pose.x + step_size/4, current_pose.y);

  //Draw r
  }else if(current_task == 14){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y + step_size/2);
  }else if(current_task == 15){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x + step_size/3, current_pose.y);  
  }else if(current_task == 16){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y - step_size/6);

  // Next Letter
  }else if(current_task == 17){
    change_pen(0, 0, 0, 5, 1);        
    set_goal(current_pose.x, current_pose.y + step_size/6); 
  }else if(current_task == 18){
    change_pen(0, 0, 0, 5, 1);        
    set_goal(current_pose.x + step_size/4, current_pose.y); 

  // Draw i
  }else if(current_task == 19){
    change_pen(0, 0, 0, 5, 1);        
    set_goal(current_pose.x, current_pose.y + 2*step_size/4);
  }else if(current_task == 20){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y - step_size/4);
  }else if(current_task == 21){
    change_pen(0, 0, 0, 5, 1);        
    set_goal(current_pose.x, current_pose.y - step_size/4);

  }else if(current_task == 22){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x, current_pose.y - step_size/2);
  }else if(current_task == 23){
    change_pen(0, 0, 0, 5, 0);        
    set_goal(current_pose.x + step_size/6, current_pose.y);                                        

  // Go to line position
  }else if(current_task == 24){
    change_pen(0, 0, 0, 5, 1);       
    set_goal(current_pose.x + step_size, current_pose.y - step_size);
  }else if(current_task == 25){
    change_pen(50, 50, 50, 2, 0);        
    set_goal(current_pose.x - 4*step_size, current_pose.y);
  }else if(current_task == 26){
    change_pen(50, 50, 50, 2, 0);        
    set_goal(current_pose.x + 4*step_size, current_pose.y);
  }else if(current_task == 27){
    change_pen(50, 50, 50, 2, 0);        
    set_goal(current_pose.x - 4*step_size, current_pose.y);
    current_task = 25;
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