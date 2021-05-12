#include <vector>
#include "pigpiod_if2.h"
#include "math.h"
#include "geometry_msgs/msg/twist.hpp"

class Motor
{
	int gpio_high, gpio_low, gpio_pwm;

public:

	Motor() {}
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

	void set_off()
		{
		gpioWrite(gpio_low, 0);
		gpioWrite(gpio_high, 0);
		gpioPWM(gpio_pwm, 0);
	}

	void set(float signal)
		{
		int pwm = (int)round(fabs(signal) * 255);
		if (pwm == 0)     set_off();
		else if (pwm > 0) set_high(pwm);
		else              set_low(pwm);
	}
	void set_high(int pwm)
		{
		gpioWrite(gpio_low, 0);
		gpioWrite(gpio_high, 1);
		gpioPWM(gpio_pwm, pwm);
	}
	void set_low(int pwm)
		{
		gpioWrite(gpio_low, 1);
		gpioWrite(gpio_high, 0);
		gpioPWM(gpio_pwm, pwm);
	}
};