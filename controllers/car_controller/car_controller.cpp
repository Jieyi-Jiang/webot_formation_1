#include <iostream>
#include <cstdio>
#include <vector>
#include <cmath>
#include <thread>

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
#include <webots/Compass.hpp>
#include <webots/InertialUnit.hpp>

#include <common/mavlink.h>

#include "mavlink_udp.hpp"
#include "pid.hpp"
#include "gamepad.h"

// #include <Eigen/Dense>

#define PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)
#define M_PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)


#define TIME_STEP 8
using namespace webots;
using namespace std;
// using Eigen::MatrixXd;

#define MAX_SPEED 0.5
#define MAX_ANGLE 0.7
#define DISTANCE_TOLERANCE 0.2
#define TARGET_POINTS_SIZE 16
typedef struct positon
{
    float x;
    float y;
} Postition;

static int current_target_index = 0;

static Postition targets[TARGET_POINTS_SIZE] = {
    {0.0, 0.0},
    {1.0, 0.5},
    {2.0, 1.5},
    {2.5, 2.5},
    {2.7, 3.7},
    {2.0, 4.5},
    {1.8, 4.8},
    {1.0, 4.5},
    {0.0, 3.5},
    {-1.0, 4.5},
    {-1.8, 4.8},
    {-2.0, 4.5},
    {-2.7, 3.7},
    {-2.5, 2.5},
    {-2.0, 1.5},
    {-1.0, 0.5},
};

static 

double inline min_max(double value, double limit){ 
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}


static int time_cnt = 0;

// car
static Car *my_car;
// m/s
double speed = 0.0;
// radian
double steering_angle = 0.0;
// brake: 0 - 1.0
double throttle = 0.0;
static double brake = 0.0;
static bool  brake_flag = false;
static float target_position[3] = {0.0, 0.0, 0.0};

// gps
static GPS *gps;
static float gps_coords[3] = {0.0, 0.0, 0.0};
static float gps_speed[3] = {0.0, 0.0, 0.0};

// accelerometer
static Accelerometer *acce;
static double acce_value[3] = {0.0, 0.0, 0.0};

// gyro
static Gyro *gyro;
static double gyro_value[3] = {0.0, 0.0, 0.0};

// compass
static Compass *compass;
static double compass_value[3] = {0.0, 0.0, 0.0};

// inertial unit
static InertialUnit *inertial_unit;
static double inertial_unit_value[3] = {0.0, 0.0, 0.0};

// keyboard
static Keyboard *keyboard;

void set_speed()
{
    if (brake_flag) speed = 0.0;
    speed = min_max(speed, MAX_SPEED);
    // cout << "speed: " << speed << endl;
    double kmh = speed * 3.6;    
    my_car->setCruisingSpeed(kmh);
}


void set_throttle()
{
    if (throttle < 0.0) throttle = 0.0;
    else if (throttle > 1.0) throttle = 1.0;
    cout << "throttle: " << throttle << endl;
    my_car->setGear(3);
    my_car->setThrottle(throttle);
}

void set_steering_angle()
{
    // cout << "steering angle: " << steering_angle << endl;
    steering_angle = min_max(steering_angle, MAX_ANGLE);
    my_car->setSteeringAngle(steering_angle);
}

void set_brake()
{
    // cout << "brake: " << brake << endl;
    brake = min_max(brake, 1.0);
    my_car->setBrakeIntensity(brake);
}

void update_gps()
{
    const double *tmp_coords = gps->getValues();
    const double *tmp_speeds = gps->getSpeedVector();
    gps_coords[0] = (float)tmp_coords[0]; // East -- x
    gps_coords[1] = (float)tmp_coords[1]; // North  -- y
    gps_coords[2] = (float)tmp_coords[2]; // Up  -- z
    gps_speed[0] = (float)tmp_speeds[0];  // East speed -- vx
    gps_speed[1] = (float)tmp_speeds[1];  // North speed  -- vy
    gps_speed[2] = (float)tmp_speeds[2];  // Up speed  -- vz
    // printf("gps: %f %f %f\n", gps_coords[0], gps_coords[1], gps_coords[2]);
}

void update_acce()
{
    const double *tmp = acce->getValues();
    acce_value[0] = tmp[0];
    acce_value[1] = tmp[1];
    acce_value[2] = tmp[2];
    // printf("acce: %f %f %f\n", acce_value[0], acce_value[1], acce_value[2]);
}

void update_gyro()
{
    const double *tmp = gyro->getValues();
    gyro_value[0] = tmp[0];
    gyro_value[1] = tmp[1];
    gyro_value[2] = tmp[2];
    // printf("gyro: %f %f %f\n", gyro_value[0], gyro_value[1], gyro_value[2]);
}

void update_inertial_unit()
{
    const double *tmp = inertial_unit->getRollPitchYaw();
    inertial_unit_value[0] = tmp[0];
    inertial_unit_value[1] = tmp[1];
    inertial_unit_value[2] = tmp[2];
}

// 罗盘指示北方，如果车头指向北方，则角度为0，如果车头指向南方，则角度为180
// 如果车头指向东方，则角度为90，如果车头指向西方，则角度为-90
void update_compass()
{
    const double *tmp = compass->getValues();
    compass_value[0] = tmp[0];  // East  -- x
    compass_value[1] = tmp[1];  // North -- y
    compass_value[2] = tmp[2];  // Up    -- z
    // printf("compass: %f %f %f\n", compass_value[0], compass_value[1], compass_value[2]);
}

void check_key()
{
    int currentKey = keyboard->getKey();
    // cout << "currentKey: " << currentKey << endl;
    switch (currentKey)
    {
    case -1:
        break;
    case Keyboard::UP:
        brake_flag = false;
        brake = 0.0;
        speed += 0.06*TIME_STEP/64;
        speed = min_max(speed, MAX_SPEED);
        // printf("speed up  ^^^^\n");
        break;
    case Keyboard::DOWN:
        brake_flag = false;
        brake = 0.0;
        speed -= 0.06*TIME_STEP/64;
        speed = min_max(speed, MAX_SPEED);
        // printf("speed down VVVV\n");
        break;
    case Keyboard::LEFT:
        steering_angle -= 0.1*TIME_STEP/64;
        steering_angle = min_max(steering_angle, MAX_ANGLE);
        // printf("turn left <<<<\n");
        break;
    case Keyboard::RIGHT:
        steering_angle += 0.1*TIME_STEP/64;
        steering_angle = min_max(steering_angle, MAX_ANGLE);
        // printf("turn right >>>>\n");
        break;
    case 'B':
    case 'b':
        brake_flag = true;
        brake += 0.1*TIME_STEP/64;
        brake = min_max(brake, 1.0);
        // printf("brake !!!!\n");
        break;
    default:
        break;
    }
}

void send_heartbeat(MavlinkUDP *mav_udp)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);

    mav_udp->mavudp_send_message(&message); 
    printf("sent heartbeat\n");
    // mavlink_msg_to_send_buffer(buffer, &message);
}


void get_target_position(const mavlink_message_t &message)
{
    // printf("get target position\n");
    mavlink_local_position_ned_t local_position_ned;
    mavlink_msg_local_position_ned_decode(&message, &local_position_ned);

    target_position[0] = local_position_ned.x;
    target_position[1] = local_position_ned.y;
    target_position[2] = local_position_ned.z;
    printf("target position: %f %f %f\n", target_position[0], target_position[1], target_position[2]);
}


void normalize_vector(double &x, double &y)
{
    double length = sqrt(x*x + y*y);
    x /= length;
    y /= length;
}

// void update_angle()
double angle_of_vector_2d(double x1, double y1, double x2, double y2)
{
    normalize_vector(x1, y1);
    normalize_vector(x2, y2);
    double cosine = x1*x2 + y1*y2;
    double sine = y2*x1-y1*x2;
    double angle = atan2(sine, cosine)/M_PI_2/2*180;
    return angle;
}

inline double norm_3d(double x, double y, double z)
{
    return sqrt(x*x + y*y + z*z);
}

inline double norm_2d(double x, double y)
{
    return sqrt(x*x + y*y);
}

PID speed_pid(0.5, 0, 0.15, 0, MAX_SPEED); // speed_pid.setTarget(0);
PID angle_pid(0.05, 0.01, 0.01, -MAX_ANGLE, MAX_ANGLE);
void car_pid()
{
    double err_position = norm_2d(target_position[0], target_position[1]) - norm_2d(gps_coords[0], gps_coords[1]);
    double err_angle = 
    angle_of_vector_2d(
        compass_value[0], compass_value[1], 
        target_position[0]-gps_coords[0], target_position[1]-gps_coords[1]);
    // double set_speed = speed_pid.update_pos(abs(err_position));
    double set_speed = 0.5;
    double set_angle = angle_pid.update_inc(err_angle);
    printf("err_position:%f err_angle:%f set_speed:%f set_angle:%f\n", err_position, err_angle, set_speed, set_angle);
    set_speed = (MAX_ANGLE - abs(set_angle) + 0.4) * set_speed;
    speed = min_max(set_speed, MAX_SPEED);
    steering_angle = min_max(set_angle, MAX_ANGLE);
}

void update_target_position()
{
    if (norm_2d(gps_coords[0] - target_position[0], gps_coords[1] - target_position[1]) < DISTANCE_TOLERANCE)
    {
        current_target_index ++;
        current_target_index %= TARGET_POINTS_SIZE;
        target_position[0] = targets[current_target_index].x;
        target_position[1] = targets[current_target_index].y;
    }
    printf("current_target_index:%d\n", current_target_index);
    printf("target_position:%f %f\n", target_position[0], target_position[1]);
}

void gamepad_control()
{
    extern SDL_Gamepad *gamepad;
    int left_x = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX);
    int left_y = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTY);
    int right_x = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTX);
    int right_y = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTY);
    // cout << "left-x: " << left_x << endl;
    // cout << "left-y: " << left_y << endl;
    // cout << "right-x: " << right_x << endl;
    // cout << "right-y: " << right_y << endl;
    double temp = -right_y / 32767.0;
    speed = temp * 0.2;

    // throttle = - right_y / 32767.0 * 0.7;
    steering_angle = left_x / 32767.0 * 0.7;
}

void thread_time()
{
    int i = 0;
    while (true)
    {
        // 输出线程测试的编号
        // 输出线程测试的编号
        // cout << "thread test #" << i << endl;
        // 增加全局计数器
        i += 1; 
        // 暂停1秒
        time_cnt += 1;
        Sleep(1);
    }
}

void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    cout << "Got heartbeat from";
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            cout << " generic";
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            cout << " ArduPilot";
            break;
        case MAV_AUTOPILOT_PX4:
            cout << " PX4";
            break;
        default:
            cout << " other";
            break;
    }
    cout << " autopilot" << endl;
}

void send_heartbeat_th(MavlinkUDP* mav_udp, const uint8_t system_id)
{
    mavlink_message_t message;
    mavlink_heartbeat_t heartbeat;
    // mavlink_status_t status;
    // const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;
    uint8_t component_id = MAV_COMP_ID_PERIPHERAL;
    heartbeat.autopilot = MAV_AUTOPILOT_PX4;
    heartbeat.base_mode = base_mode;
    heartbeat.custom_mode = custom_mode;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.type = MAV_TYPE_GENERIC;
    heartbeat.mavlink_version = 3;
    mavlink_msg_heartbeat_encode(system_id, component_id, &message, &heartbeat);
    // mavlink_msg_heartbeat_pack_chan(
    //     *system_id,
    //     MAV_COMP_ID_PERIPHERAL,
    //     MAVLINK_COMM_0,
    //     &message,
    //     MAV_TYPE_GENERIC,
    //     MAV_AUTOPILOT_PX4,
    //     base_mode,
    //     custom_mode,
    //     MAV_STATE_STANDBY);
    while (true)
    {
        mav_udp->mavudp_send_message(&message);
        cout << time_cnt << ": send heatbeat" << endl;
        Sleep(1000);
    }
}

void recive_date_th(MavlinkUDP* mav_udp, const uint8_t system_id)
{
    mavlink_message_t message;
    mavlink_status_t status;
    mavlink_local_position_ned_t data1;
    data1.x = 0;
    data1.y = 0;
    while (true)
    {
        mav_udp->mavudp_recive_message(MAVLINK_COMM_0, &message, &status);
        if (message.sysid != 0) continue;
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // handle_heartbeat(&message);
            handle_heartbeat(&message);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            mavlink_msg_local_position_ned_decode(&message, &data1);
            double velocity_ctrl = data1.x;
            double angle_ctrl = data1.y;
            cout << "Control: " << (int)message.compid << endl;
            cout << "User: " << int(system_id + MAV_COMP_ID_USER1 - 1) << endl;
            if (message.compid == system_id + MAV_COMP_ID_USER1 - 1)
            {

                speed = velocity_ctrl;
                steering_angle = angle_ctrl;
                cout << int(system_id)  << " | velocity_ctrl: " << velocity_ctrl << " angle_ctrl: " << angle_ctrl << endl;
            }
            break;
        }
    }
}

double compass2angle(double N, double E) // N = y, E = x
{   
    double arctan = atan2(N, E);
    // if (N > 0 && E < 0)
    // {
    //     arctan = arctan + PI;
    // }
    // else if (N < 0 && E < 0)
    // {
    //     arctan = arctan - PI;
    // }
    return arctan;
}

void send_data(MavlinkUDP *mav_udp, uint8_t system_id)
{
    mavlink_message_t message;

    // const uint8_t system_id = 1;
    // const uint8_t base_mode = 0;
    // const uint8_t custom_mode = 0;
    // uint8_t comp_id = MAV_COMP_ID_GPS;
    while (true)
    {
        mavlink_msg_local_position_ned_pack_chan(
            system_id,
            MAV_COMP_ID_GPS,
            MAVLINK_COMM_0,
            &message,
            0,
            gps_coords[0],
            gps_coords[1],
            gps_coords[2],
            gps_speed[0],
            gps_speed[1],
            gps_speed[2]
        );
        mav_udp->mavudp_send_message(&message);
        enum axis{roll=0, pitch=1, yaw=2};
        float angle[3];
        angle[roll] =  0;
        angle[pitch] = 0;
        // 错误的
        // angle[roll] =  compass2angle(atan2f(compass_value[2], compass_value[1]));
        // angle[pitch] = compass2angle(atan2f(compass_value[2], compass_value[0]));
        // 罗盘指示北方，顺时针为正，当数值为0时，说明在正北方向，90度为正东，180度为正南，270度为正西
        // angle[yaw] =   compass2angle(atan2f(compass_value[1], compass_value[0]));
        //  East - x  为零度，逆时针旋转
        // angle[yaw] = compass2angle(atan2f(compass_value[1], compass_value[0]))-1.570796327;
        angle[yaw] = compass2angle(compass_value[0], compass_value[1]);
        // cout << "angle: " << angle[yaw] << endl;
        // Sleep(1);
        mavlink_msg_local_position_ned_system_global_offset_pack_chan(
            system_id,
            MAV_COMP_ID_GPS2,
            MAVLINK_COMM_0,
            &message,
            0,
            compass_value[0],
            compass_value[1],
            compass_value[2],
            angle[roll],
            angle[pitch],
            angle[yaw]
        );
        mav_udp->mavudp_send_message(&message);
        mavlink_highres_imu_t data;
        data.time_usec = 0;
        data.xacc = (float)acce_value[0];
        data.yacc = (float)acce_value[1];
        data.zacc = (float)acce_value[2];
        data.xgyro = (float)gyro_value[0];
        data.ygyro = (float)gyro_value[1];
        data.zgyro = (float)gyro_value[2];
        data.xmag = (float)inertial_unit_value[0];
        data.ymag = (float)inertial_unit_value[1];
        data.zmag = (float)inertial_unit_value[2];
        data.abs_pressure = (float)(my_car->getCurrentSpeed() / 3.6);
        data.diff_pressure = (float)(my_car->getSteeringAngle());
        mavlink_msg_highres_imu_encode(system_id, MAV_COMP_ID_IMU, &message, &data);
        mav_udp->mavudp_send_message(&message);
        Sleep(10);
    }
}

// ip, mul_port, my_port, car_num
int main(int argc, char *argv[])
{
    for (int i = 0; i < argc; ++ i)
    {
        cout << "argv" << i << " :" << argv[i] << endl;
    }
    SDL_init();
    // printf("SDL init\n");
    cout << "SDL init" << endl;
    SDL_AddEventWatch(&my_SDL_event, NULL);
    cout << "begin" << endl;
    my_car = new Car();
    const int basicTimeStep = int(my_car->getBasicTimeStep());
    const int sensorTimeBase = 10;
    const int sensorTimeStep = sensorTimeBase * basicTimeStep;
    gps = my_car->getGPS("GPS");
    gps->enable(sensorTimeStep);
    acce = my_car->getAccelerometer("accelerometer");
    acce->enable(sensorTimeStep);
    gyro = my_car->getGyro("gyro");
    gyro->enable(sensorTimeStep);
    compass = my_car->getCompass("compass");
    compass->enable(sensorTimeStep);
    inertial_unit = my_car->getInertialUnit("inertial unit");
    inertial_unit->enable(sensorTimeStep);
    keyboard = my_car->getKeyboard();
    keyboard->enable(sensorTimeStep);
    // my_car.setCon
    MavlinkUDP mav_udp(argv[1], argv[2], argv[3]);
    mavlink_message_t message;
    uint8_t car_id = (uint8_t)atol(argv[4]);
    // LPVOID param_arr[2] = {&udp_port, &message};
    // HANDLE hThread = CreateThread(NULL, 0, udp_rcv_func, (LPVOID)&udp_port, 0, NULL);
    // printf("create thread\n");
    thread th_time(thread_time);
    thread th_send_gps(send_data, &mav_udp, car_id);
    thread th_re_heart(recive_date_th, &mav_udp, car_id);
    thread th_se_heart(send_heartbeat_th, &mav_udp, car_id);
    SDL_Event event;
    while (my_car->step() != -1)
    {
        // check_key();
        // gamepad_control();
        while (SDL_PollEvent(&event))
        {
            // cout << "event: " << event.type << endl;
            handle_SDL_event(&event);
        }
        update_gps();
        update_acce();
        update_gyro();
        update_acce();
        update_compass();
        update_inertial_unit();
        // send_gps(&mav_udp);
        // update_target_position();
        // car_pid();
        set_speed();
        // set_throttle();
        set_steering_angle();
        // printf("speed :%lf\n", speed);
        // printf("steering_angle :%lf\n", steering_angle);
        // printf("angle :%lf\n", angle_of_vector_2d(compass_value[0], compass_value[1], target_position[0]-gps_coords[0], target_position[1]-gps_coords[1]));
        // udp_port.read_message(message);
        // printf("msgid: %d\n", message.msgid);
        // switch (message.msgid)
        // {
        //     case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        //         get_target_position(message);
        //         // memset(&g_message_read, 0, sizeof(g_message_read));
        //         break;
        //     case 0:
        //         get_target_position(message);
        //         // memset(&g_message_read, 0, sizeof(g_message_read));
        //         break;
        //     default:
        //         break;
        // }
        // printf("========================================================\n");
    }
    SDL_quit();
    delete my_car;
    th_time.join();
    th_send_gps.join();
    th_re_heart.join();
    th_se_heart.join();
    printf("end\n");
    return 0;
}



