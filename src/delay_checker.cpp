#include <ros/ros.h>
#include <delay_checker/delay_checker.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>

#define RPM_PER_LINEAR 35

namespace delay_checker
{
	DelayChecker::DelayChecker(const ros::NodeHandle &nh): _nh(nh)
	{
		std::string input_topic, output_topic;
		_nh.param("input_topic", input_topic, std::string("/cmd_vel"));	
		_nh.param("output_topic", output_topic, std::string("/output"));

		input_sub = _nh.subscribe(input_topic, 1, &DelayChecker::inputCallback, this);	
		output_sub = _nh.subscribe(output_topic, 1, &DelayChecker::outputCallback, this);
		pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		message = R"(
			w: moving forward    ( +1 m/s )
			s: moving backward   ( -1 m/s )
			a: turn left         ( +1 rad/s )
			d: turn right        ( -1 rad/s )
			----------------------------------
			Enter: Publishing to Twist
			anything else: stop
			CTRL-C to quit
		)";

		// Initialize
		terminate = false;
		key_input_done = false;
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;

		speed = 0.0;
		turn = 0.0;
		prev_speed = 0.0;
	}

	void DelayChecker::spinNode()
	{
		ros::Rate rate(20);
		while (_nh.ok()){
			ros::spinOnce();

			if(!terminate) {
				printf("%s\n", message);
				interface();
				pub.publish(twist);
				start = std::chrono::steady_clock::now();
				terminate = true;
			}

			rate.sleep();
		}
	}

	void DelayChecker::inputCallback(const geometry_msgs::Twist& msg)
	{
		prev_speed = msg.linear.x;
	}

	void DelayChecker::outputCallback(const erp42_communication::ERP42_feedback& msg)
	{
		if(msg.speed_rpm >= threshhold || fabs(msg.steer_degree) >= 0.1) {
				auto end_speed = std::chrono::steady_clock::now();
				std::chrono::duration<double> diff_speed = end_speed - start;

				printf("\n\n-----[msg recived]-----\n");
				printf("threshhold %f\n", threshhold);
				printf("speed %f, steer %f : pub to sub took %f sec\n", msg.speed_rpm, msg.steer_degree, diff_speed.count());
				terminate = false;
		}
	}

	int DelayChecker::getch()
	{
		int ch;
		struct termios oldt;
		struct termios newt;

		// Store old settings, and copy to new settings
		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;

		// Make required changes and apply the settings
		newt.c_lflag &= ~(ICANON | ECHO);
		newt.c_iflag |= IGNBRK;
		newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
		newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
		newt.c_cc[VMIN] = 1;
		newt.c_cc[VTIME] = 0;
		tcsetattr(fileno(stdin), TCSANOW, &newt);

		// Get the current character
		ch = getchar();

		// Reapply old settings
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

		return ch;		
	}

	void DelayChecker::interface()
	{
		printf("\nAwaiting command...\n");
		
		// Reset speed and turn values for next command
		speed = turn = 0.0;

		while(true){

			// Get the pressed key
			key = getch();

			// If the key corresponds to a key in the terminal instruction
			if (key == '\x77'){
				speed += 1.0;
				printf("\rCurrent: speed %f\tturn %f | Last command: %c", speed, turn, key);
			}
			else if (key == '\x73'){
				speed -= 1.0;
				printf("\rCurrent: speed %f\tturn %f | Last command: %c", speed, turn, key);
			}
			else if (key == '\x61'){
				turn += 1.0;
				printf("\rCurrent: speed %f\tturn %f | Last command: %c", speed, turn, key);
			}
			else if (key == '\x64'){
				turn -= 1.0;
				printf("\rCurrent: speed %f\tturn %f | Last command: %c", speed, turn, key);
			}
			// Publish the twist message
			else if (key == '\x0D'){
				printf("\nENTER pressed!\n");
				// Forbid backward direction
				if (speed < 0) speed = 0.0;

				// Update
				twist.linear.x = speed; twist.linear.y = 0.0; twist.linear.z = 0.0;
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = turn;

				threshhold = (speed > 1 && prev_speed != 0) ? prev_speed * RPM_PER_LINEAR + 0.5 : 0.5;

				break;
			}
			else if (key == '\x03'){
				// If ctrl-C (^C) was pressed, terminate the program
				printf("\nCtrl-C pressed!\n");
				_nh.shutdown();
				terminate = true;
				break;
			}
			// Otherwise, set the robot to stop
			else {
				speed = 0.0;
				turn = 0.0;
				printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
			}
		}
   key_input_done = true;
	}


}	// namespace delay_checker