#include <cstdlib>
#include <iostream>
#include <ros/ros.h>

using namespace std;

const int increase = 2;

class Armlog {
	double* arm_log_;
	double* time_log_;
	int  size, capacity;

public:
	//Constructor
	Armlog(){
		size = 0;
		capacity = 100;
		arm_log_ = new double[capacity];
		time_log_ =  new double[capacity];
	}

	void increase_capacity();
	void addtolog(double log_input, double time_input);
	void printlog(string s);

};
