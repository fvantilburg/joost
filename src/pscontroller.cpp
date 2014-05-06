/*
 * pscontroller.cpp

 *
 *  Created on: 20 Nov 2013
 *      Author: martijm
 */
//$ sudo bash
//$ rosrun ps3joy ps3joy.py

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sensor_msgs/JoyFeedback.h>

class Controller {
    ros::NodeHandle nh_;
    int store1_;
    double max_speed_;
    int speed_help_up_, speed_help_down_, Upper_speed_, Lower_speed_, Shoulder_speed_,arm_init_help_,move_help_, move_help2_;
    ros::Subscriber joy_sub_;
    ros::Publisher vel_pub_, arm_pub_, calibrate_pub_,status_pub_;
	geometry_msgs::Twist velocity;
	geometry_msgs::Twist arm_movement;
	geometry_msgs::Twist arm_start;
	geometry_msgs::Twist calibrate;
	geometry_msgs::Twist status_request;

public:
    Controller(): store1_(0), Upper_speed_(0), Lower_speed_(0), Shoulder_speed_(0){};
    void init();
    void spin();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};
void Controller::init(){

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("joystick",10);
    arm_pub_ = nh_.advertise<geometry_msgs::Twist>("joy_arm",10);
    calibrate_pub_ = nh_.advertise<geometry_msgs::Twist>("calibrate_arm",10);
    status_pub_ = nh_.advertise<geometry_msgs::Twist>("update_request",10);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Controller::joyCallback, this);


    max_speed_ = 0;
    Upper_speed_ = 0;
    Lower_speed_ = 0;

    speed_help_up_ = 0;
    speed_help_down_ = 0;

    move_help_ = 0;
    move_help2_ = 0;

    arm_init_help_ = 0;
  
}




void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

    //--------------------------------------------BASE------------------------//

    // Speed up - button 11
    if(joy->buttons[11] == 1 && max_speed_ <= 0.6){
    	speed_help_up_ = 1;
    }
    if(joy->buttons[11] == 0 && speed_help_up_ == 1){
    	max_speed_ = max_speed_+0.1;
    	speed_help_up_ = 0;
    }

    // Speed down - button 9
    if(joy->buttons[9] == 1 && max_speed_ >= 0.1){
    	speed_help_down_ = 1;
    }
    if(joy->buttons[9] == 0 && speed_help_down_ == 1){
    	max_speed_ = max_speed_-0.1;
    	speed_help_down_ = 0;
    }
    
    // Stop - button 14 - X
    if(joy->buttons[14] == 1){
    	max_speed_ = 0;
    }
    
    // Move the base - left stick
    velocity.linear.x = max_speed_*joy->axes[1];
    velocity.angular.z = 0.7*max_speed_*joy->axes[0];
    
    //--------------------------------------------ARM-------------------------//
    
    // Upper arm up - button 4, Upper arm down - button 6
    if(joy->buttons[4] == 1 && joy->buttons[6] == 0){
        Upper_speed_ = 1;
    }
    else if(joy->buttons[4] == 0 && joy->buttons[6] == 1){
        Upper_speed_ = -1;
    }
    else{
        Upper_speed_ = 0;
    }
    
      
    // Lower arm up - button 10, Lower arm down - button 8
    if(joy->buttons[10] == 1 && joy->buttons[8] == 0){
        Lower_speed_ = 1;
    }
    else if(joy->buttons[10] == 0 && joy->buttons[8] == 1){
        Lower_speed_ = -1;
    }
    else{
        Lower_speed_ = 0;
    }
    
    //--------------------------------------------MOVE------------------------//

    // Request data button 15, the square
    if(joy->buttons[15] == 1){
       	move_help_ = 1;
    }

    if(joy->buttons[15] == 0 && move_help_ == 1){
       	move_help_ = 0;
       	status_pub_.publish(status_request);
    }

    // the circle
    if(joy->buttons[13] == 1){
        	move_help2_ = 1;
        }

        if(joy->buttons[13] == 0 && arm_init_help_ == 1){
        	arm_movement.linear.z = 1;
        	move_help2_ = 0;
        }
        else{
        	arm_movement.linear.z = 0;
        }

    //--------------------------------------------INITS-----------------------//

    // Init Arm - button 12, the triangle
    if(joy->buttons[12] == 1){
    	arm_init_help_ = 1;
    }

    if(joy->buttons[12] == 0 && arm_init_help_ == 1){
    	calibrate.linear.x = 1;
    	arm_init_help_ = 0;
    	ROS_INFO("%s", "WOoooooo doet ie t al de calibreer ding?");
    	calibrate_pub_.publish(calibrate);
    }
    else{
    	calibrate.linear.x = 0;
    }





    // Publish to the base
    vel_pub_.publish(velocity);

    // Publish to the arm
    arm_movement.angular.x = Upper_speed_;                //Speed of the upper (shoulder) arm
    arm_movement.angular.y = Lower_speed_;               //Speed of the lower (elbow) arm
    arm_pub_.publish(arm_movement);


    //arm_init_pub_.publish(arm_start);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "PScontroller");
  Controller PScont;
  PScont.init();
  ros::spin();
}


