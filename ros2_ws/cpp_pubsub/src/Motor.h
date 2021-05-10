#include <vector>
#include <lib/pigpio.h>

class Motor
{
	int gpio_high, gpio_low, gpio_pwm;

public:

	Motor() {}
	Motor(int gpio_high, int gpio_low, int gpio_pwm);
	~Motor();

	void set_off();
	void set(float64 signal);
	void set_high(int pwm);
	void set_low(int pwm);
};