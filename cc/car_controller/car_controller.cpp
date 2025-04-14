#include <iostream>

#include <webots/Device.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Robot.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>

#define TIME_STEP 8

// car
webots::Car my_car;
// m/s
double speed = 0.0;
// radian
double steering_angle = 0.0;

// gps
webots::GPS *gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// accelerometer
webots::Accelerometer *acce;
double acce_value[3] = {0.0, 0.0, 0.0};

// gyro
webots::Gyro *gyro;
double gyro_value[3] = {0.0, 0.0, 0.0};

// keyboard
webots::Keyboard *keyboard;

void set_speed()
{
    if (speed > 3.0)
        speed = 3.0;
    else if (speed < -3.0)
        speed = -3.0;
    double kmh = speed * 3.6;
    speed = kmh;
    my_car.setCruisingSpeed(kmh);
}

void set_steering_angle()
{
    if (steering_angle > 1.5)
        steering_angle = 1.5;
    else if (steering_angle < -1.5)
        steering_angle = -1.5;
    my_car.setSteeringAngle(steering_angle);
}

void update_gps()
{
    const double *tmp = gps->getValues();
    gps_coords[0] = tmp[0];
    gps_coords[1] = tmp[1];
    gps_coords[2] = tmp[2];
}

void check_key()
{
    int currentKey = keyboard->getKey();
    switch (currentKey)
    {
    case -1:
        break;
    case webots::Keyboard::UP:
        speed += 0.1;
        break;
    case webots::Keyboard::DOWN:
        speed -= 0.1;
        break;
    case webots::Keyboard::LEFT:
        steering_angle -= 0.1;
        break;
    case webots::Keyboard::RIGHT:
        steering_angle += 0.1;
        break;
    default:
        break;
    }
}

int main()
{
    printf("begin\n");
    int basicTimeStep = int(my_car.getBasicTimeStep());
    int sensorTimeStep = 4 * basicTimeStep;
    my_car = webots::Car();
    gps = my_car.getGPS("GPS");
    gps->enable(sensorTimeStep);
    acce = my_car.getAccelerometer("accelerometer");
    acce->enable(sensorTimeStep);
    gyro = my_car.getGyro("gyro");
    gyro->enable(sensorTimeStep);
    keyboard = my_car.getKeyboard();
    keyboard->enable(sensorTimeStep);

    while (my_car.step() != -1)
    {
        check_key();
        set_speed();
        set_steering_angle();
    }
    my_car.~Car();
    printf("end\n");
    return 0;
}
