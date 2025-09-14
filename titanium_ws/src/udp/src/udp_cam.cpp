#include "ros/ros.h"
#include "robot_msgs/camera.h"

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

robot_msgs::camera cam;

ros::Timer timer_udpWrite;
ros::Time last_cam_time;
double TIMEOUT = 0.5;

ros::Subscriber sub_cam;


void timeoutCallback()
{
    if((ros::Time::now() - last_cam_time).toSec() > TIMEOUT)
    {
        robot_msgs::camera zero_cam;

        cam = zero_cam;
    }
}

void udpWriteCallback(const ros::TimerEvent &event)
{
    timeoutCallback();

    // memcpy(tx_buffer +  3, &robot_system.start, 1);
    // memcpy(tx_buffer +  4, &robot_system.reset, 1);
	// memcpy(tx_buffer +  5, &motor_base.motor_a, 2);
    // memcpy(tx_buffer +  7, &motor_base.motor_b, 2);
    // memcpy(tx_buffer +  9, &motor_base.motor_c, 2);
    // memcpy(tx_buffer + 11, &motor_arm.motor_1, 2);
    // memcpy(tx_buffer + 13, &motor_arm.motor_2, 2);
    // memcpy(tx_buffer + 15, &motor_arm.motor_3, 2);
    // memcpy(tx_buffer + 17, &relay.relay_state, 1);
	
    socklen_t len = sizeof(cliaddr);
	sendto(sockfd, tx_buffer, sizeof(tx_buffer), MSG_CONFIRM,
			(const struct sockaddr *) &cliaddr, len); 
}

void cameraCallback(const robot_msgs::cameraPtr &msg)
{
    last_cam_time = ros::Time::now();
	cam = *msg;
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
    
    sub_cam = nh.subscribe("/sensor/camera", 10, cameraCallback);

    timer_udpWrite = nh.createTimer(ros::Duration(0.001), udpWriteCallback);
    
    spinner.start();

    ros::waitForShutdown();

    spinner.stop();

    close(sockfd);

    return 0;
}