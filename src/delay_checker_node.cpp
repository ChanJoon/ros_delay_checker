#include <ros/ros.h>
#include <delay_checker/delay_checker.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "delay_checker");
  ros::NodeHandle nh("~");

	delay_checker::DelayChecker dc(nh);
	dc.spinNode();

  return(0);
}