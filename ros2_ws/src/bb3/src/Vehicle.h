#include "Motor.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Vehicle : public rclcpp::Node
{
	int pi;
	std::string name;
	Motor left_top, left_bottom, right_top, right_bottom;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;

	std::vector<Motor> get_motors()
	{
		return {left_top, left_bottom, right_top, right_bottom};
	}

	void telemetry_callback(geometry_msgs::msg::Twist::SharedPtr telemetry)
	{
		// Get velocity and rotation
		float velocity = telemetry->linear.x;
		float rotation = telemetry->angular.z;

		// Truncate
		velocity = std::max(0.0f, std::min(1.0f, velocity));
		rotation = std::max(-1.0f, std::min(1.0f, rotation));

		// Translate to motor signals
		// Turn to the left -> slow down right motors
		if (rotation > 0.0f)
		{
			right_top.set(velocity * (1 - rotation));
			right_bottom.set(velocity * (1 - rotation));
			left_top.set(velocity);
			left_bottom.set(velocity);
		}
		// Turn to the right -> slow down left motors
		else
		{
			right_top.set(velocity);
			right_bottom.set(velocity);
			left_top.set(velocity * (1 - fabs(rotation)));
			left_bottom.set(velocity * (1 - fabs(rotation)));
		}
	}

public:

	// Ctors and dtors
	Vehicle(std::string name, Motor left_top, Motor left_bottom, Motor right_top, Motor right_bottom)
		: Node("vehicle"), name(name),
		left_top(left_top), left_bottom(left_bottom), 
		right_top(right_top), right_bottom(right_bottom)
	{
		// Create a subscription to the telemetry topic
		subscription = this->create_subscription
		<geometry_msgs::msg::Twist>(                     // message class
			"cmd_vel",                    // topic name
			10,                                     // message queue depth
			std::bind(
				&Vehicle::telemetry_callback, // callback function
				this,                                   // ??
				std::placeholders::_1));                // ??
	}

	~Vehicle()
	{
		stop();
	}

	// Testing
	void voltage_test(float duration)
	{
		left_top.set_high(80);
		time_sleep(duration);
	}
	
	void test()
	{
		std::vector<Motor> motors = get_motors();
		for (Motor m : motors)
		{
			m.set_high(100);
			time_sleep(1.0);
			stop();
			time_sleep(0.5);
			m.set_low(200);
			time_sleep(1.0);
			stop();
			time_sleep(0.5);
			m.set_high(255);
			time_sleep(1.0);
			stop();
			time_sleep(1.0);
		}
	}

	// Movement control
	void stop()
	{
		left_top.set_off();
		left_bottom.set_off();
		right_top.set_off();
		right_bottom.set_off();
	}

	void stop(float delay)
	{
		stop();
		time_sleep(delay);
	}

	void forward(int pwm, float delay)
	{
		left_top.set_high(pwm);
		left_bottom.set_high(pwm);
		right_top.set_high(pwm);
		right_bottom.set_high(pwm);
		time_sleep(delay);
	}
	
	void backward(int pwm, float delay)
	{
		left_top.set_low(pwm);
		left_bottom.set_low(pwm);
		right_top.set_low(pwm);
		right_bottom.set_low(pwm);
		time_sleep(delay);
	}

	void left(int pwm, float delay)
	{
		left_top.set_high(pwm);
		left_bottom.set_high(pwm);
		right_top.set_low(pwm);
		right_bottom.set_low(pwm);
		time_sleep(delay);
	}

	void right(int pwm, float delay)
	{
		left_top.set_low(pwm);
		left_bottom.set_low(pwm);
		right_top.set_high(pwm);
		right_bottom.set_high(pwm);
		time_sleep(delay);
	}
};