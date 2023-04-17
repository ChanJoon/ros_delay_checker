#include <ros/ros.h>
#include <delay_checker/delay_checker.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

namespace delay_checker
{
	DelayChecker::DelayChecker(const ros::NodeHandle &nh): _nh(nh)
	{
		std::string input_topic, output_topic;
		_nh.param("input_topic", input_topic, std::string("/cmd_vel"));	
		_nh.param("output_topic", output_topic, std::string("/output"));

		input_sub = _nh.subscribe(input_topic, 10, &DelayChecker::inputCallback, this);	
		output_sub = _nh.subscribe(output_topic, 10, &DelayChecker::outputCallback, this);
		pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

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
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;

		speed = 0.0;
		turn = 0.0;

	}

	void DelayChecker::spinNode()
	{
		ros::Rate rate(50);
		while (_nh.ok()){
			pub.publish(twist);
			ros::spinOnce();

			if(!terminate) printf("%s\n", message);
			interface();

			rate.sleep();
		}
	}

	void DelayChecker::inputCallback(const geometry_msgs::Twist& msg)
	{
		// Currently not used
		// auto end = std::chrono::steady_clock::now();
		// std::chrono::duration<double> diff = end - start;

		// printf("\nmsg subscribed!");
		// printf("\npub to sub took %f sec\n", diff.count());
	}

	void DelayChecker::outputCallback(const erp42_communication::ERP42_feedback& msg)
	{
		auto end = std::chrono::steady_clock::now();
		std::chrono::duration<double> diff = end - start;

		printf("\nmsg subscribed!");
		printf("\npub to sub took %f sec\n", diff.count());
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
				else if (turn < 0) turn = 0.0;

				// Update
				twist.linear.x = speed; twist.linear.y = 0.0; twist.linear.z = 0.0;

				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = turn;

				start = std::chrono::steady_clock::now();
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
	}


}	// namespace delay_checker