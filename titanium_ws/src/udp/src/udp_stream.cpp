#include "ros/ros.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/ultrasonic.h"

ros::Publisher pub_encoder, pub_ultrasonic;
ros::Timer timer_1ms;
robot_msgs::encoder encoder;
robot_msgs::ultrasonic ultrasonic;

void timer1msCallback(const ros::TimerEvent &event)
{
    encoder.enc_a += 1;
    encoder.enc_b += 2;
    encoder.enc_c += 3;
    encoder.enc_x += 4;
    encoder.enc_y += 5;

    pub_encoder.publish(encoder);

    ultrasonic.ultrasonic_a += 10;
    ultrasonic.ultrasonic_b += 20;
    ultrasonic.ultrasonic_c += 30;
    ultrasonic.ultrasonic_d += 40;
    
    pub_ultrasonic.publish(ultrasonic);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_node");
    
    ros::NodeHandle nh;

    pub_encoder = nh.advertise<robot_msgs::encoder>("/sensor/encoder", 10);
    pub_ultrasonic = nh.advertise<robot_msgs::ultrasonic>("/sensor/ultrasonic", 10);

    timer_1ms = nh.createTimer(ros::Duration(0.001), timer1msCallback);

    ROS_INFO("Publisher Node Started!");

    ros::spin();
    
    return 0;
}


// #include "ros/ros.h"
// #include "std_msgs/Float32MultiArray.h"

// #include <cstring>
// #include <unistd.h>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>

// #define STM32_ADDR 	"192.168.13.111"
// #define STM32_PORT	1555
// #define PC_ADDR		"192.168.13.100"
// #define PC_PORT    	1556
// #define BUFF_SIZE 	64

// typedef struct
// {
// 	int16_t enc_a;
// 	int16_t enc_b;
// 	int16_t enc_c;

// 	int16_t enc_x;
// 	int16_t enc_y;

// 	float 	yaw_degree;

// 	uint16_t ultrasonic[4];

// } udpRx_t ;

// typedef struct
// {
// 	int16_t motor_a;
// 	int16_t motor_b;
// 	int16_t motor_c;
// } udpTx_t ;

// udpTx_t udp_tx;
// udpRx_t udp_rx;

// int sockfd;
// char rx_buffer[BUFF_SIZE];
// char tx_buffer[BUFF_SIZE] = "ABC";
// struct sockaddr_in servaddr, cliaddr;

// ros::Timer timer_1ms;
// ros::Publisher pub;
// ros::Subscriber sub;


// void timerCallback1ms(const ros::TimerEvent &event)
// {
// 	socklen_t len = sizeof(cliaddr);
// 	int bytes_captured = recvfrom(sockfd, rx_buffer, BUFF_SIZE, MSG_DONTWAIT, 
// 						 (struct sockaddr *)&cliaddr, &len);

// 	if (bytes_captured > 0)
// 	{
// 		memcpy(&udp_rx, rx_buffer +  3, sizeof(udpRx_t));

// 		ROS_INFO("ENC A,B,C: %d; %d; %d", udp_rx.enc_a, udp_rx.enc_b, udp_rx.enc_c);
// 		ROS_INFO("ENC X,Y: %d; %d", udp_rx.enc_x, udp_rx.enc_y);
// 		ROS_INFO("YAW DEG: %.2f", udp_rx.yaw_degree);
// 		ROS_INFO("ULTRA 0,1,2,3: %d; %d; %d; %d", udp_rx.ultrasonic[0], udp_rx.ultrasonic[1], udp_rx.ultrasonic[2], udp_rx.ultrasonic[3]);
// 		ROS_INFO("==============================");

// 		std_msgs::Float32MultiArray msg;

// 		msg.data.clear();
// 		msg.data.push_back((float)udp_rx.enc_a);
// 		msg.data.push_back((float)udp_rx.enc_b);
// 		msg.data.push_back((float)udp_rx.enc_c);
// 		msg.data.push_back((float)udp_rx.enc_x);
// 		msg.data.push_back((float)udp_rx.enc_y);
// 		msg.data.push_back((float)udp_rx.yaw_degree);
// 		msg.data.push_back((float)udp_rx.ultrasonic[0]);
// 		msg.data.push_back((float)udp_rx.ultrasonic[1]);
// 		msg.data.push_back((float)udp_rx.ultrasonic[2]);
// 		msg.data.push_back((float)udp_rx.ultrasonic[3]);
		
// 		pub.publish(msg);
// 	}

// }

// void dataSubCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
// {
// 	udp_tx.motor_a = (int16_t)msg->data[0];
// 	udp_tx.motor_b = (int16_t)msg->data[1];
// 	udp_tx.motor_c = (int16_t)msg->data[2];
	
// 	memcpy(tx_buffer +  3, &udp_tx, sizeof(udpTx_t));
	
//     socklen_t len = sizeof(cliaddr);
// 	sendto(sockfd, tx_buffer, sizeof(tx_buffer), MSG_CONFIRM,
// 			(const struct sockaddr *) &cliaddr, len); 
// }

// void initSocket()
// {
//     if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
//     {
//         ROS_ERROR("SOCKET CREATION FAILED!!!");
//         exit(EXIT_FAILURE);
//     }
    
//     memset(&servaddr, 0, sizeof(servaddr));
//     servaddr.sin_family = AF_INET; 
//     servaddr.sin_addr.s_addr = inet_addr(PC_ADDR);
//     servaddr.sin_port = htons(PC_PORT);

//     memset(&cliaddr, 0, sizeof(cliaddr));
//     cliaddr.sin_family = AF_INET;
//     cliaddr.sin_addr.s_addr = inet_addr(STM32_ADDR);
//     cliaddr.sin_port = htons(STM32_PORT);
    
//     if(bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
//     {
//         ROS_ERROR("BIND FAILED!!!");
//         close(sockfd);
//         exit(EXIT_FAILURE);
//     }

//     ROS_INFO("UDP Server running on %s:%d", inet_ntoa(servaddr.sin_addr), PC_PORT);
// }

// int main(int argc, char **argv) 
// {
//     ros::init(argc, argv, "udp_node");

//     ros::NodeHandle nh;

//     ros::AsyncSpinner spinner(0);

//     initSocket();

//     pub = nh.advertise<std_msgs::Float32MultiArray>("/stm32_to_ros", 100);
//     sub = nh.subscribe("/ros_to_stm32", 100, dataSubCallback);
//     timer_1ms = nh.createTimer(ros::Duration(0.001), timerCallback1ms);

//     spinner.start();

//     ros::waitForShutdown();

//     close(sockfd);

//     return 0;
// }