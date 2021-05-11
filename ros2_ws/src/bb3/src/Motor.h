#include <vector>
#include "lib/pigpio.h"
#include "geometry_msgs/msg/twist.hpp"

class Motor
{
	int gpio_high, gpio_low, gpio_pwm;

public:

	Motor();
	Motor(int gpio_high, int gpio_low, int gpio_pwm)
	{   
		this->gpio_high = gpio_high;
		this->gpio_low = gpio_low;
		this->gpio_pwm = gpio_pwm;
		gpioSetMode(gpio_high, PI_OUTPUT);
		gpioSetMode(gpio_low, PI_OUTPUT);
		gpioSetMode(gpio_pwm, PI_OUTPUT);
		set_off();
	}
	//~Motor();

	void set_off();
	void set(float signal);
	void set_high(int pwm);
	void set_low(int pwm);
};