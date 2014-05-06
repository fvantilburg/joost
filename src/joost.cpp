#include "joost/joost.h"
#include "joost/Arm.h"
#include <XMLConfiguration.h>
#include <threemxl/dxlassert.h>
#include <unistd.h>



//int const Number_of_joints = 4;
double const    L1 = 0,
                L2 = 0.3475,
                L3 = 0.33015,
                L4 = 0;


void Joost::init()
{
	ROS_INFO("Initializing Arm");
	initialization_done_ = false;

	ROS_INFO("Reading Parameters");

	// Read parameters
	ROS_ASSERT(nh_.getParam("motor_port", motor_port_name));
	ROS_ASSERT(nh_.getParam("motor_config_arm", motor_config_name));

	ROS_INFO("Parameters have been read");

	// Load motor configuration
	CXMLConfiguration motor_config_xml;
	ROS_ASSERT(motor_config_xml.loadFile(motor_config_name));

	ROS_INFO("Motor configuration loaded");

	CDxlConfig motor_config_elbow;
	motor_config_elbow.readConfig(motor_config_xml.root().section("elbow"));
	elbow_ = new C3mxlROS(motor_port_name.c_str());
	elbow_->setConfig(&motor_config_elbow);

	ROS_INFO("Elbow motor loaded");

	CDxlConfig motor_config_shoulder;
	motor_config_shoulder.readConfig(motor_config_xml.root().section("shoulder"));
	shoulder_ = new C3mxlROS(motor_port_name.c_str());
	shoulder_->setConfig(&motor_config_shoulder);

	ROS_INFO("Shoulder motor loaded");



	ros::Rate init_rate(1);


	while(ros::ok() && elbow_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize elbow motor, will continue trying every second");    		//Initializing connection
		init_rate.sleep();
	}
	ROS_INFO("Elbow motor initialized");

	while(ros::ok() && shoulder_->init() != DXL_SUCCESS)
	{
		ROS_WARN_ONCE("Couldn't initialize shoulder motor, will continue trying every second");		//Initializing connection
		init_rate.sleep();
	}
	ROS_INFO("Shoulder motor initialized");

	shoulder_accel_ = 0.7;

	calibration_ = false;


	shoulder_->set3MxlMode(SPEED_MODE);
	shoulder_->setAcceleration(shoulder_accel_);

	elbow_->set3MxlMode(SPEED_MODE);
	elbow_->setAcceleration(elbow_accel_);

	mode_ = 2;

	// Advertise and publish on topics
	pos_arm_sub_ = nh_.subscribe("pos_arm", 1, &Joost::posCallback, this);
	joy_arm_sub_ = nh_.subscribe("joy_arm", 1, &Joost::joyarmCallback, this);
	calibrate_sub_ = nh_.subscribe("calibrate_arm", 1, &Joost::armcalibrateCallback, this);

	ROS_INFO("%s", "Init is done!");
}

void Joost::spin()
{
	ros::Rate loop_rate(50);
	ROS_INFO("%s", "Spinning");
	while(ros::ok()){
		shoulder_->getState();
		shoulder_->getStatus();

		elbow_->getState();
		elbow_->getStatus();

		ros::spinOnce();
		loop_rate.sleep();
	}

}

void Joost::armcalibrateCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if(calibration_ == false){
		calibration_ = true;
		ros::Rate calibrate_rate(50);

		mode_ = 1;
		ROS_INFO("%s", "Start initialization");

		//Calibrate the shoulder
		shoulder_->set3MxlMode(EXTERNAL_INIT);
		shoulder_->setSpeed(0.1);
		shoulder_->setAcceleration(1);
		shoulder_->setTorque(1);

		while (ros::ok())
		{
			// Check for status
			shoulder_->getStatus();

			shoulder_stat_ = shoulder_->presentStatus();


			if (shoulder_stat_ != M3XL_STATUS_INITIALIZE_BUSY)
			{
				break;
			}

			calibrate_rate.sleep();
		}

		if (shoulder_stat_ != M3XL_STATUS_INIT_DONE)
		{
			ROS_FATAL_STREAM("Couldn't initialize shoulder: " << shoulder_->translateErrorCode(shoulder_stat_));
			ROS_ISSUE_BREAK();
		}



		//Calibrate the elbow
		elbow_->set3MxlMode(EXTERNAL_INIT);
		elbow_->setSpeed(-0.1);
		elbow_->setAcceleration(1);
		elbow_->setTorque(1);

		while (ros::ok())
		{
			// Check for status
			elbow_->getStatus();
			elbow_stat_ = elbow_->presentStatus();


			if (elbow_stat_ != M3XL_STATUS_INITIALIZE_BUSY)
			{
				break;
			}

			calibrate_rate.sleep();
		}

		if (elbow_stat_ != M3XL_STATUS_INIT_DONE)
		{
			ROS_FATAL_STREAM("Couldn't initialize shoulder: " << elbow_->translateErrorCode(elbow_stat_));
			ROS_ISSUE_BREAK();
		}

		shoulder_->set3MxlMode(SPEED_MODE);
		elbow_->set3MxlMode(SPEED_MODE);

		shoulder_->setAcceleration(0.8);
		elbow_->setAcceleration(0.8);

	}


}

void Joost::joyarmCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

	if(mode_ != 2){
		shoulder_->set3MxlMode(SPEED_MODE);
		elbow_->set3MxlMode(SPEED_MODE);
		mode_ = 2;
	}

	shoulder_->setSpeed((msg->angular.y)*(-0.2));
	elbow_->setSpeed((msg->angular.x)*(-0.2));




	if(msg->linear.y == 1){
		if(mode_ != 0){
			shoulder_->set3MxlMode(POSITION_MODE);
			shoulder_->setAcceleration(0.7);
			shoulder_->setSpeed(0.2);

			elbow_->set3MxlMode(POSITION_MODE);
			elbow_->setAcceleration(0.7);
			elbow_->setSpeed(0.2);

			mode_ = 0;
		}

		shoulder_->getPos();
		elbow_->getPos();

		current_shoulder_pos_ = shoulder_->presentPos();
		current_elbow_pos_ = elbow_->presentPos();

		shoulder_->setPos(current_shoulder_pos_ + 0.5);
		elbow_->setPos(current_elbow_pos_ + 0.5);
	}


}



void Joost::posCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	if(mode_ != 0){
		double shoulder_speed_ = 0.1, shoulder_accel_ = 0.7, elbow_speed_ = 0.1, elbow_accel_ = 0.7;

		shoulder_->set3MxlMode(POSITION_MODE);
		shoulder_->setAcceleration(shoulder_accel_);
		shoulder_->setSpeed(shoulder_speed_);

		elbow_->set3MxlMode(POSITION_MODE);
		elbow_->setAcceleration(elbow_accel_);
		elbow_->setSpeed(elbow_speed_);

		mode_ = 0;
	}

	goal_shoulder_ = (-1)*(msg->angular.x-calibrate_shoulder_pos_);
	goal_elbow_ = (-1)*(msg->angular.y-calibrate_elbow_pos_);

	if((calibration_ == 1)){
		shoulder_->setPos(goal_shoulder_);
		elbow_->setPos(goal_elbow_);

		shoulder_->getState();
		shoulder_->getStatus();

		elbow_->getState();
		elbow_->getStatus();

		current_shoulder_pos_ = (-1)*(shoulder_->presentPos()-calibrate_shoulder_pos_);
		current_elbow_pos_ = (-1)*(elbow_->presentPos()-calibrate_elbow_pos_);

		std::cout<< "Shoulder =" << current_shoulder_pos_ << "\t" << "Goal = " << msg->angular.x << "\n"
				<< "Elbow = " << current_elbow_pos_ << "\t" << "Goal = " << msg->angular.y << "\n" << "\n";
	}
}

matrix<C3mxlROS*, NUMBER_OF_JOINTS,1> Joost::getMotors(){

	matrix<C3mxlROS*, NUMBER_OF_JOINTS,1> motors_;
	motors_(0) = shoulder_;
	motors_(1) = elbow_;

	return motors_;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Joost");

  Joost joost;
  joost.init();


  Arm arm_joost;
  cout << "Joost created" << endl;
  arm_joost.init(joost.getMotors());
  cout << "Joost initialized" << endl;
  arm_joost.test();
  cout<< "Joost tested" << endl;
  joost.spin();


  ros::spin();
  return 0;
}
