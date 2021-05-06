#include <pigpio.h>

#define LENAA 26
#define LIN1 19
#define LIN2 13
#define LIN3 6
#define LIN4 5
#define LENAB 11
#define RENAA 21
#define RIN1 20
#define RIN2 16
#define RIN3 12
#define RIN4 7
#define RENAB 8

class motor
{
	int gpio_high, gpio_low, gpio_ena;

public:

	motor() {}
	motor(int gpio_high, int gpio_low, int gpio_ena)
	{
		this->gpio_high = gpio_high;
		this->gpio_low = gpio_low;
		this->gpio_ena = gpio_ena;
		gpioSetMode(gpio_high, PI_OUTPUT);
		gpioSetMode(gpio_low, PI_OUTPUT);
		gpioSetMode(gpio_ena, PI_OUTPUT);
		set_off();
	}

	~motor()
	{
		set_off();
	}

	void set_off()
	{
		gpioWrite(gpio_low, 0);
		gpioWrite(gpio_high, 0);
		gpioWrite(gpio_ena, 0);
	}

	void set_high()
	{
		gpioWrite(gpio_low, 0);
		gpioWrite(gpio_high, 1);
		gpioWrite(gpio_ena, 1);
	}
	
	void set_low()
	{
		gpioWrite(gpio_low, 1);
		gpioWrite(gpio_high, 0);
		gpioWrite(gpio_ena, 1);
	}
};

class vehicle
{
	motor left_top, left_bottom, right_top, right_bottom;

public:

	vehicle() {}
	vehicle(motor left_top, motor left_bottom, motor right_top, motor right_bottom)
	{
		this->left_top = left_top;
		this->left_bottom = left_bottom;
		this->right_top = right_top;
		this->right_bottom = right_bottom;
	}

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

	void forward(float delay)
	{
		left_top.set_high();
		left_bottom.set_high();
		right_top.set_high();
		right_bottom.set_high();
		time_sleep(delay);
	}

	void backward(float delay){
		left_top.set_low();
		left_bottom.set_low();
		right_top.set_low();
		right_bottom.set_low();
		time_sleep(delay);
	}
	void left(float delay)
	{
		left_top.set_high();
		left_bottom.set_high();
		right_top.set_low();
		right_bottom.set_low();
		time_sleep(delay);
	}
	void right(float delay)
	{
		left_top.set_low();
		left_bottom.set_low();
		right_top.set_high();
		right_bottom.set_high();
		time_sleep(delay);
	}
};

int main()
{
	gpioInitialise();
	vehicle robot(motor(RIN1, RIN2, RENAA), motor(RIN4, RIN3, RENAB),
		      motor(LIN3, LIN4, LENAB), motor(LIN2, LIN1, LENAA));

	robot.forward(2.5);
	robot.stop(1.5);
	robot.backward(2);
	robot.stop();
}
