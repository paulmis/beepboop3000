#include "Motor.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/Twist.h>

using namespace std::chrono_literals;

class Vehicle : public rclcpp::Node
{
	std::string name;
	Motor left_top, left_bottom, right_top, right_bottom;
	rrclcpp::Subscription<geometry_msgs::Twist>::SharedPtr subscription;

	std::vector<motor> get_motors();
	void telemetry_callback();

public:

	// Ctors and dtors
	Vehicle() {}
	Vehicle(motor left_top, motor left_bottom, motor right_top, motor right_bottom);
	~Vehicle();

	// Testing
	void voltage_test(float duration);
	void test();

	// Movement control
	void stop();
	void stop(float delay);
	void forward(int pwm, float delay);
	void backward(int pwm, float delay);
	void left(int pwm, float delay);
	void right(int pwm, float delay);
}
