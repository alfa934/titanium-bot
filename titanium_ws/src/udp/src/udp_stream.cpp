#include "ros/ros.h"
#include "robot_msgs/button.h"
#include "robot_msgs/controller.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/indicator.h"
#include "robot_msgs/limit_switch.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/robot_system.h"
#include "robot_msgs/ultrasonic.h"
#include "robot_msgs/yaw.h"

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define STM32_ADDR 	"192.168.13.111"
#define STM32_PORT	1555
#define PC_ADDR		"192.168.13.100"
#define PC_PORT    	1556
#define BUFF_SIZE 	81

int sockfd;
char rx_buffer[BUFF_SIZE];
char tx_buffer[BUFF_SIZE] = "ABC";
struct sockaddr_in servaddr, cliaddr;

robot_msgs::button button;
robot_msgs::controller controller;
robot_msgs::encoder encoder;
robot_msgs::indicator indicator;
robot_msgs::limit_switch limit_switch;
robot_msgs::motor motor_base;
robot_msgs::motor motor_arm;
robot_msgs::motor relay;
robot_msgs::robot_system robot_system;
robot_msgs::ultrasonic ultrasonic;
robot_msgs::yaw yaw;

ros::Timer timer_udpRead, timer_udpWrite;
ros::Timer timer_timeout;
ros::Time last_arm_time, last_base_time, last_relay_time, last_system_time, last_indicator_time;
const double TIMEOUT = 0.5;

ros::Publisher  pub_encoder, pub_limit, pub_ultra, pub_yaw,
                pub_button, pub_controller;

ros::Subscriber sub_motor_base, sub_motor_arm, sub_relay, sub_robot_system, sub_indicator;

void udpReadCallback(const ros::TimerEvent &event)
{
	socklen_t len = sizeof(cliaddr);
	int bytes_captured = recvfrom(sockfd, rx_buffer, BUFF_SIZE, MSG_DONTWAIT, 
						 (struct sockaddr *)&cliaddr, &len);

	if (bytes_captured > 0)
	{
		memcpy(&encoder.enc_a, rx_buffer + 3, 2);
		memcpy(&encoder.enc_b, rx_buffer + 5, 2);
		memcpy(&encoder.enc_c, rx_buffer + 7, 2);
		memcpy(&encoder.enc_x, rx_buffer + 9, 2);
		memcpy(&encoder.enc_y, rx_buffer + 11, 2);
        memcpy(&encoder.enc_1, rx_buffer + 13, 2);
        memcpy(&encoder.enc_2, rx_buffer + 15, 2);
        memcpy(&encoder.enc_3, rx_buffer + 17, 2);
		
        memcpy(&yaw.degree, rx_buffer + 19, 4);
        yaw.radian = yaw.degree * (180.0 / M_PI);
		
        memcpy(&ultrasonic.ultra_a, rx_buffer + 23, 2);
		memcpy(&ultrasonic.ultra_b, rx_buffer + 25, 2);
		memcpy(&ultrasonic.ultra_c, rx_buffer + 27, 2);
		memcpy(&ultrasonic.ultra_d, rx_buffer + 29, 2);

        memcpy(&limit_switch.lim_2, rx_buffer + 31, 1);
        memcpy(&limit_switch.lim_3, rx_buffer + 32, 1);

        memcpy(&button.startButton, rx_buffer + 33, 1);
        memcpy(&button.resetButton, rx_buffer + 34, 1);
        memcpy(&button.button1, rx_buffer + 35, 1);
        memcpy(&button.button2, rx_buffer + 36, 1);
        memcpy(&button.button3, rx_buffer + 37, 1);
        memcpy(&button.button4, rx_buffer + 38, 1);
        memcpy(&button.button5, rx_buffer + 39, 1);

        button.startButton = !button.startButton;
        button.resetButton = !button.resetButton;
        button.button1 = !button.button1;
        button.button2 = !button.button2;
        button.button3 = !button.button3;
        button.button4 = !button.button4;
        button.button5 = !button.button5;

        // if(button.resetButton == 0)
        // {
        //     robot_system.reset = 1;
        // }
        // else
        // {
        //     robot_system.reset = 0;
        // }

        memcpy(&controller.rX, rx_buffer + 40, 1);
        memcpy(&controller.rY, rx_buffer + 41, 1);
        memcpy(&controller.lX, rx_buffer + 42, 1);
        memcpy(&controller.lY, rx_buffer + 43, 1);
        memcpy(&controller.r2, rx_buffer + 44, 1);
        memcpy(&controller.l2, rx_buffer + 45, 1);
        memcpy(&controller.r1, rx_buffer + 46, 1);
        memcpy(&controller.l1, rx_buffer + 47, 1);
        memcpy(&controller.r3, rx_buffer + 48, 1);
        memcpy(&controller.l3, rx_buffer + 49, 1);
        memcpy(&controller.crs, rx_buffer + 50, 1);
        memcpy(&controller.sqr, rx_buffer + 51, 1);
        memcpy(&controller.tri, rx_buffer + 52, 1);
        memcpy(&controller.cir, rx_buffer + 53, 1);
        memcpy(&controller.up, rx_buffer + 54, 1);
        memcpy(&controller.down, rx_buffer + 55, 1);
        memcpy(&controller.right, rx_buffer + 56, 1);
        memcpy(&controller.left, rx_buffer + 57, 1);
        memcpy(&controller.share, rx_buffer + 58, 1);
        memcpy(&controller.option, rx_buffer + 59, 1);
        memcpy(&controller.ps, rx_buffer + 60, 1);
        memcpy(&controller.touchpad, rx_buffer + 61, 1);
        memcpy(&controller.battery, rx_buffer + 62, 1);
        memcpy(&controller.gX, rx_buffer + 63, 2);
        memcpy(&controller.gY, rx_buffer + 65, 2);
        memcpy(&controller.gZ, rx_buffer + 67, 2);
        memcpy(&controller.aX, rx_buffer + 69, 2);
        memcpy(&controller.aY, rx_buffer + 71, 2);
        memcpy(&controller.aZ, rx_buffer + 73, 2);


        pub_button.publish(button);
        pub_controller.publish(controller);
		pub_encoder.publish(encoder);
		pub_limit.publish(limit_switch);
		pub_ultra.publish(ultrasonic);
		pub_yaw.publish(yaw);
	}
}

void udpWriteCallback(const ros::TimerEvent &event)
{
    memcpy(tx_buffer +  3, &robot_system.start, 1);
    memcpy(tx_buffer +  4, &robot_system.reset, 1);
	memcpy(tx_buffer +  5, &motor_base.motor_a, 2);
    memcpy(tx_buffer +  7, &motor_base.motor_b, 2);
    memcpy(tx_buffer +  9, &motor_base.motor_c, 2);
    memcpy(tx_buffer + 11, &motor_arm.motor_1, 2);
    memcpy(tx_buffer + 13, &motor_arm.motor_2, 2);
    memcpy(tx_buffer + 15, &motor_arm.motor_3, 2);
    memcpy(tx_buffer + 17, &relay.relay_state, 1);

    for(int i = 0; i < 10; i++)
    {
        memcpy(tx_buffer + 18 + i, &indicator.indicator[i], 1);
    }
	
    socklen_t len = sizeof(cliaddr);
	sendto(sockfd, tx_buffer, sizeof(tx_buffer), MSG_CONFIRM,
			(const struct sockaddr *) &cliaddr, len); 
}

void timeoutCallback(const ros::TimerEvent& event)
{
    if((ros::Time::now() - last_base_time).toSec() > TIMEOUT)
    {
        motor_arm.motor_a = 0;
        motor_arm.motor_b = 0;
        motor_arm.motor_c = 0;
    }
    if((ros::Time::now() - last_arm_time).toSec() > TIMEOUT)
    {
        motor_arm.motor_1 = 0;
        motor_arm.motor_2 = 0;
        motor_arm.motor_3 = 0;
    }
    if((ros::Time::now() - last_relay_time).toSec() > TIMEOUT)
    {
        motor_arm.relay_state = 0;
    }
    if((ros::Time::now() - last_system_time).toSec() > TIMEOUT)
    {
        robot_system.reset = 0;
        robot_system.start = 0;
    }
    if((ros::Time::now() - last_indicator_time).toSec() > TIMEOUT)
    {
        for(int i = 0; i < 10; i++)
        {
            indicator.indicator[i] = 0;
        }
        
    }
}

void motorBaseCallback(const robot_msgs::motorConstPtr &msg)
{
    last_base_time = ros::Time::now();
	motor_base = *msg;
}

void motorArmCallback(const robot_msgs::motorConstPtr &msg)
{
    last_arm_time = ros::Time::now();
    motor_arm = *msg;
}

void relayCallback(const robot_msgs::motorConstPtr &msg)
{
    last_relay_time = ros::Time::now();
    relay = *msg;
}

void robotSystemCallback(const robot_msgs::robot_systemConstPtr &msg)
{
    last_system_time = ros::Time::now();
    robot_system = *msg;
}

void indicatorCallback(const robot_msgs::indicatorConstPtr &msg)
{
    last_indicator_time = ros::Time::now();
    indicator = *msg;
}

void initSocket()
{
    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        ROS_ERROR("SOCKET CREATION FAILED!!!");
        exit(EXIT_FAILURE);
    }
    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET; 
    servaddr.sin_addr.s_addr = inet_addr(PC_ADDR);
    servaddr.sin_port = htons(PC_PORT);

    memset(&cliaddr, 0, sizeof(cliaddr));
    cliaddr.sin_family = AF_INET;
    cliaddr.sin_addr.s_addr = inet_addr(STM32_ADDR);
    cliaddr.sin_port = htons(STM32_PORT);
    
    if(bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("BIND FAILED!!!");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    ROS_INFO("UDP Server running on %s:%d", inet_ntoa(servaddr.sin_addr), PC_PORT);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "udp_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);

    initSocket();

    pub_button = nh.advertise<robot_msgs::button>("/input/button", 10);
    pub_controller = nh.advertise<robot_msgs::controller>("/input/controller", 10);
    pub_encoder = nh.advertise<robot_msgs::encoder>("/sensor/encoder", 10);
	pub_limit = nh.advertise<robot_msgs::limit_switch>("/sensor/limit_switch", 10);
	pub_ultra = nh.advertise<robot_msgs::ultrasonic>("/sensor/ultrasonic", 10);
	pub_yaw = nh.advertise<robot_msgs::yaw>("/sensor/yaw", 10);
    
    sub_motor_base = nh.subscribe("/actuator/motor/base", 10, motorBaseCallback);
    sub_motor_arm = nh.subscribe("/actuator/motor/arm", 10, motorArmCallback);
    sub_relay = nh.subscribe("/actuator/relay", 10, relayCallback);
    sub_robot_system = nh.subscribe("/system/robot_system", 10, robotSystemCallback);
    sub_indicator = nh.subscribe("/system/indicator", 10, indicatorCallback);

    timer_udpRead = nh.createTimer(ros::Duration(0.001), udpReadCallback);
    timer_udpWrite = nh.createTimer(ros::Duration(0.001), udpWriteCallback);
    timer_timeout = nh.createTimer(ros::Duration(0.001), timeoutCallback);
    
    spinner.start();

    ros::waitForShutdown();

    close(sockfd);

    return 0;
}