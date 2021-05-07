#include <vector>
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
	int gpio_high, gpio_low, gpio_pwm;

public:

	motor() {}
	motor(int gpio_high, int gpio_low, int gpio_pwm)
	{
		this->gpio_high = gpio_high;
		this->gpio_low = gpio_low;
		this->gpio_pwm = gpio_pwm;
		gpioSetMode(gpio_high, PI_OUTPUT);
		gpioSetMode(gpio_low, PI_OUTPUT);
		gpioSetMode(gpio_pwm, PI_OUTPUT);
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
		gpioPWM(gpio_pwm, 0);
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

class vehicle
{
	motor left_top, left_bottom, right_top, right_bottom;
	
	std::vector<motor> get_motors()
	{
		return {left_top, left_bottom, right_top, right_bottom};
	}

public:

	vehicle() {}
	vehicle(motor left_top, motor left_bottom, motor right_top, motor right_bottom)
	{
		this->left_top = left_top;
		this->left_bottom = left_bottom;
		this->right_top = right_top;
		this->right_bottom = right_bottom;
	}
	
	~vehicle()
	{
		stop();
	}

	void voltage_test(float duration)
	{
		left_top.set_high(80);
		time_sleep(duration);
	}

	void test()
	{
		std::vector<motor> motors = get_motors();
		for (motor m : motors)
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

	void backward(int pwm, float delay){
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

int main()
{
	gpioInitialise();
	vehicle robot(motor(RIN1, RIN2, RENAA), motor(RIN4, RIN3, RENAB),
		      motor(LIN3, LIN4, LENAB), motor(LIN2, LIN1, LENAA));

	robot.stop();
	robot.test();
	robot.stop();
}
