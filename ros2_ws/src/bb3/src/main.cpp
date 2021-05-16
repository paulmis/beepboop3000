#include "Vehicle.h"
#include "pigpiod_if2.h"

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
    // Initialize pigpiod_if2
    int pi = pigpio_start(nullptr, nullptr);

    // If the assigned raspberrypi number is non-negative,
    // the initialization succeeded
    if (pi >= 0)
    {
	std::cout << "pigpiod_if2 initialized successfully" << std::endl;
        rclcpp::init(argc, argv);
	std::cout << "spinning the vehicle...!" << std::endl;
        rclcpp::spin(
            std::make_shared<Vehicle>(
                pi, "beepboop3000", 
                Motor(pi, RIN1, RIN2, RENAA), Motor(pi, RIN4, RIN3, RENAB),
                Motor(pi, LIN3, LIN4, LENAB), Motor(pi, LIN2, LIN1, LENAA)));
        rclcpp::shutdown();
    }
    else 
    {
        std::cout << "pigpiod_if2 initialization failed: code " << pi << std::endl;
    }    
}
