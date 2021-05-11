#include "Vehicle.h"

std::vector<Motor> Vehicle::get_motors()
{
    return {left_top, left_bottom, right_top, right_bottom};
}

// Dummy ctor
Vehicle::Vehicle() {}

// Constructs a new 4-wheeled vehicle that listens to telemetry input on '$name-telemetry'
Vehicle::Vehicle(std::string name, Motor left_top, Motor left_bottom, Motor right_top, Motor right_bottom)
    : Node("vehicle-" + name), name(name),
    left_top(left_top), left_bottom(left_bottom), right_top(right_top), right_bottom(right_bottom)
{
    // Create a subscription to the telemetry topic
    subscription = this->create_subscription
    <geometry_msgs::Twist>(                     // message class
        name + "-telemetry",                    // topic name
        10,                                     // message queue depth
        std::bind(&Vehicle::telemetry_callback, // callback function
        this,                                   // ??
        std::placeholders::_1));                // ??
    )
}

// Stops the vehicle
Vehicle::~Vehicle()
{
    stop();
}

// Telemetry subscription callback
// Automatically adjusts vehicle's velocity and direction
Vehicle::telemetry_callback(geometry_msgs::msg::Twist telemetry)
{
    // Get velocity and rotation
    std_msgs::Float64 velocity = telemetry.linear.x;
    std_msgs::Float64 = telemetry.angular.z;

    // Truncate
    velocity = std::max(0.0, std::min(1.0, velocity));
    rotation = std::max(-1.0, std::min(1.0, rotation));

    // Translate to motor signals
    // Turn to the left -> slow down right motors
    if (rotation > 0.0)
    {
        right_top.set(velocity * (1 - rotation));
        right_bottom.set(velocity * (1 - rotation));
        left_top.set(velocity);
        left_bottom.set(velocity);
    }
    // Turn to the right -> slow down left motors
    else
    {
        right_top.set(velocity);
        right_bottom.set(velocity);
        left_top.set(velocity * (1 - fabs(rotation)));
        left_bottom.set(velocity * (1 - fabs(rotation)));
    }
}

void Vehicle::voltage_test(float duration)
{
    left_top.set_high(80);
    time_sleep(duration);
}

// Tests all wheels at different speeds (100, 200, 255) for a short
// period of time
void Vehicle::test()
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

void Vehicle::stop()
{
    left_top.set_off();
    left_bottom.set_off();
    right_top.set_off();
    right_bottom.set_off();
}

void Vehicle::stop(float delay)
{
    stop();
    time_sleep(delay);
}

void Vehicle::forward(int pwm, float delay)
{
    left_top.set_high(pwm);
    left_bottom.set_high(pwm);
    right_top.set_high(pwm);
    right_bottom.set_high(pwm);
    time_sleep(delay);
}

void Vehicle::backward(int pwm, float delay){
    left_top.set_low(pwm);
    left_bottom.set_low(pwm);
    right_top.set_low(pwm);
    right_bottom.set_low(pwm);
    time_sleep(delay);
}

void Vehicle::left(int pwm, float delay)
{
    left_top.set_high(pwm);
    left_bottom.set_high(pwm);
    right_top.set_low(pwm);
    right_bottom.set_low(pwm);
    time_sleep(delay);
}

void Vehicle::right(int pwm, float delay)
{
    left_top.set_low(pwm);
    left_bottom.set_low(pwm);
    right_top.set_high(pwm);
    right_bottom.set_high(pwm);
    time_sleep(delay);
}