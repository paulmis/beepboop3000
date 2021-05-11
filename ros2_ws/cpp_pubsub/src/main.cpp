#include "Vehicle.h"
#include "lib/pigpio.h"

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
#define RENAB 25

int main(int argc, char * argv[])
{
    gpioInitialise();

    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<Vehicle>(
            "beepboop3000", 
            motor(RIN1, RIN2, RENAA), motor(RIN4, RIN3, RENAB),
		    motor(LIN3, LIN4, LENAB), motor(LIN2, LIN1, LENAA)));
    rclcpp::shutdown();
}
