#include <vector>
#include "pigpiod_if2.h"
#include "math.h"
#include "geometry_msgs/msg/twist.hpp"

class Motor
{
	int pi, gpio_high, gpio_low, gpio_pwm;

public:

	Motor() {}
	Motor(int pi, int gpio_high, int gpio_low, int gpio_pwm)
		: pi(pi), gpio_high(gpio_high), gpio_low(gpio_low), gpio_pwm(gpio_pwm)
	{   
		set_mode(pi, gpio_high, PI_OUTPUT);
		set_mode(pi, gpio_low, PI_OUTPUT);
		set_mode(pi, gpio_pwm, PI_OUTPUT);
		set_off();
	}

	~Motor()
	{
		set_off();
	}

	void set_off()
	{
		gpio_write(pi, gpio_low, 0);
		gpio_write(pi, gpio_high, 0);
		set_PWM_dutycycle(pi, gpio_pwm, 0);
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
		gpio_write(pi, gpio_low, 0);
		gpio_write(pi, gpio_high, 1);
		set_PWM_dutycycle(pi, gpio_pwm, pwm);
	}

	void set_low(int pwm)
	{
		gpio_write(pi, gpio_low, 1);
		gpio_write(pi, gpio_high, 0);
		set_PWM_dutycycle(pi, gpio_pwm, pwm);
	}
};