#include "Motor.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Vehicle : public rclcpp::Node
{
	std::string name;
	Motor left_top, left_bottom, right_top, right_bottom;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;

	std::vector<Motor> get_motors();
	void telemetry_callback();

public:

	// Ctors and dtors
	Vehicle();
	Vehicle(std::string name, Motor left_top, Motor left_bottom, Motor right_top, Motor right_bottom);
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
};
