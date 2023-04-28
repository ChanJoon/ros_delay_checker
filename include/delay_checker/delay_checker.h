#ifndef _DELAY_CHECKER_H_
#define _DELAY_CHECKER_H_

#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "erp42_communication/ERP42_feedback.h"

namespace delay_checker
{
	class DelayChecker
	{
		public:

			DelayChecker(const ros::NodeHandle &nh);
			void spinNode();

		private:

			std::chrono::steady_clock::time_point start;
			const char* message;

			int key;
			bool terminate;
      bool key_input_done;
			double threshhold;

			ros::NodeHandle _nh;
			ros::Publisher pub;
			ros::Subscriber input_sub, output_sub;

			geometry_msgs::Twist twist;
			float speed;	// Linear velocity (m/s)
			float turn;		// Angular velocity (rad/s)
			float prev_speed;

			void inputCallback(const geometry_msgs::Twist& msg);
			void outputCallback(const erp42_communication::ERP42_feedback& msg);
			void interface();
			int getch();

	};	// class DelayChecker
}	// namespace delay_checker



#endif