#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

class Target {
		ros::NodeHandle nh_;
		ros::Publisher target_pub_;

		geometry_msgs::Vector3 target_;

		double x, y, z;

public:
		//Constructor
		Target(){

		}

		//Destructor
		~Target()
		{

		}
		void init();
		void send();

};

void Target::init(){

	target_pub_ = nh_.advertise<geometry_msgs::Vector3>("target_arm",10);
}

void Target::send(){
	while(ros::ok){
		cout << "Give a target" << endl;
		cout << "x = ";
		cin >> target_.x;

		cout << endl << "y = ";
		cin >> target_.y;

		cout << endl;
		target_.z = 0;

		if(target_.x == 666){
			break;
		}
		else if(sqrt(target_.x*target_.x + target_.y*target_.y) > 0.68){
			cout << "Unable to reach the desired target" << endl << endl;
		}
		else if(sqrt(target_.x*target_.x + target_.y*target_.y) < 0.3){
			cout << "Unable to reach the desired target" << endl << endl;
		}
		else{
			target_pub_.publish(target_);
		}




	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Target");
	Target two_dof;
	two_dof.init();
	two_dof.send();

}
