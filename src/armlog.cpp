#include "joost/armlog.h"

void Armlog::increase_capacity(){
	capacity = increase * capacity;
		double* temp = new double[capacity];
		double* temp2 = new double[capacity];

		for(int i = 0; i < size; i++){
			temp[i] = arm_log_[i];
			temp2[i] = time_log_[i];
		}

		arm_log_ = temp;
		time_log_ = temp2;

}

void Armlog::addtolog(double log_input, double time_input){
	if(size == capacity) increase_capacity();

	arm_log_[size] = log_input;
	time_log_[size] = time_input;
	size++;
};

void Armlog::printlog(string s){

	string filename = s + ".txt";

	ofstream fout(filename);

	for(int i = 0; i < size; i++){
		fout << arm_log_[i] << " ";
	}

	fout << endl;

	for(int i = 0; i < size; i++){
		fout << time_log_[i] << " ";
	}

	fout.close();
};

/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Armlog");

  Armlog test;
  test.addtolog(1.3);
  test.addtolog(3.3);
  test.addtolog(4.4);
  test.printlog("test");


  return 0;
}
*/
