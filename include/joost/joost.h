#ifndef JOOST_H_
#define JOOST_H_

#include <ros/ros.h>

#include <threemxl/C3mxlROS.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>

#include "dlib/matrix.h"
#include "dlib/matrix/matrix_abstract.h"

#include "joost/Arm.h"
#include "joost/armlog.h"

using namespace dlib;
using namespace std;

//int const NUMBER_OF_JOINTS = 2;

class Joost{

	ros::NodeHandle nh_;	// ROS node handle
	C3mxlROS *shoulder_;
	C3mxlROS *elbow_;
	LxSerial serial_port_;
	std::string motor_port_name, motor_config_name;

	ros::Subscriber joy_arm_sub_;
	ros::Subscriber pos_arm_sub_;
	ros::Subscriber moveto_arm_sub_;
	ros::Subscriber calibrate_sub_;

	int shoulder_stat_, elbow_stat_, mode_;
	double shoulder_speed_, shoulder_accel_, elbow_speed_, elbow_accel_, init_help_, elbow_offset;
	double current_shoulder_pos_, current_elbow_pos_, calibrate_shoulder_pos_, calibrate_elbow_pos_, goal_shoulder_, goal_elbow_;
	bool initialization_done_, mode_pos_,movement, calibration_;



	//matrix<C3mxlROS*, NUMBER_OF_JOINTS,1> motors;



public:
	// Constructor
	Joost(){

	}

	// Destructor
	~Joost()
	{
		delete shoulder_;
		delete elbow_;
		nh_.shutdown();
	}

	void init();
	void spin();
	void armcalibrateCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void joyarmCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void movetoarmCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void posCallback(const geometry_msgs::Twist::ConstPtr &msg);
	matrix<C3mxlROS*, NUMBER_OF_JOINTS,1> getMotors();

};




#endif
