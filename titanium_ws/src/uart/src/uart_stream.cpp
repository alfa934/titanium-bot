#include "ros/ros.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/ultrasonic.h"
#include "robot_msgs/motor.h"

#include <iostream>
#include <cstring>
#include "uart/serialib.h"

serialib serial;
char tx_buffer[65]= "ABC";
char rx_buffer[62];

ros::Publisher pub_encoder, pub_ultrasonic;
ros::Subscriber sub_motor;
ros::Timer timer_serialWrite, timer_serialRead;

robot_msgs::encoder encoder;
robot_msgs::ultrasonic ultrasonic;


void publishData()
{
    memcpy(&ultrasonic.ultrasonic_a, rx_buffer, 2);
    memcpy(&ultrasonic.ultrasonic_b, rx_buffer + 2, 2);
    memcpy(&ultrasonic.ultrasonic_c, rx_buffer + 4, 2);
    memcpy(&ultrasonic.ultrasonic_d, rx_buffer + 6, 2);

    memcpy(&encoder.enc_a, rx_buffer + 8, 2);
    memcpy(&encoder.enc_b, rx_buffer + 10, 2);
    memcpy(&encoder.enc_c, rx_buffer + 12, 2);
    memcpy(&encoder.enc_x, rx_buffer + 14, 2);
    memcpy(&encoder.enc_y, rx_buffer + 16, 2);

    pub_ultrasonic.publish(encoder);
    pub_encoder.publish(encoder);
}


void serialReadCallback(const ros::TimerEvent& event)
{
    static double last_serial_time = ros::Time::now().toSec();

    try
    {
        if(serial.available())
        {
            // ROS_INFO_STREAM("buffer: " << serial.available());
            static int receive_status;
            static int header_status;
            char data, data_head[1];

            if(header_status == 0)
            { 
                if(serial.available() >= 3 + sizeof(rx_buffer)) //// 3 untuk header 't', 'e', 'l'
                { 
                    for(int i=0; i<3+sizeof(rx_buffer); i++)
                    { 
                        if(serial.readBytes(data_head, 1, 1, 1) >= 1)
                        {
                            data = data_head[0];
                        }

                        if(receive_status == 0 && data == 'A')
                        {
                            receive_status++;
                        }
                        else if(receive_status == 1 && data == 'B')
                        {
                            receive_status++;
                        }
                        else if(receive_status == 2 && data == 'C')
                        {
                            receive_status = 0;
                            header_status = 1;
                            break;
                        }
                    }
                }
            }

            if(header_status == 1)
            { 
                char data_real[sizeof(rx_buffer)];

                if(serial.available() >= sizeof(rx_buffer))
                {
                    if(serial.readBytes(data_real, sizeof(rx_buffer), 1, 1) >= 1)
                    {
                        memcpy(&rx_buffer, data_real, sizeof(rx_buffer));
                    }

                    publishData();

                    header_status = 0;
                }
            }

            last_serial_time = ros::Time::now().toSec();
        }
    }
    catch (...)
    {
        encoder = robot_msgs::encoder();
        ultrasonic = robot_msgs::ultrasonic();
    }

    //--- if serial disconnects
    if(ros::Time::now().toSec() - last_serial_time > 0.5)
    {
        try
        {
            new (&serial) serialib;
            serial.openDevice("/dev/ttyUSB-SERIAL", 115200);

            if(serial.isDeviceOpen())
            {
                ROS_INFO_STREAM("serial communication restarted");
            }
        }
        catch(...)
        {
            ROS_WARN_STREAM("serial communication failed. Try reconnect in 2 seconds");
        }

        encoder = robot_msgs::encoder();
        ultrasonic = robot_msgs::ultrasonic();
        
        last_serial_time = ros::Time::now().toSec();
    }
    }


void serialWriteCallback(const ros::TimerEvent &event)
{
    for(int i = 0; i < sizeof(tx_buffer); i++)
    {
        if(serial.isDeviceOpen())
        {
            try
            {
                serial.writeChar(tx_buffer[i]);
            }
            catch(...)
            {
                ROS_WARN_STREAM("Unable to send serial!");
            }
        }
    }
}

void subMotorCallback(const robot_msgs::motorConstPtr &msg)
{
    memcpy(tx_buffer + 3, &msg->motor_a, 2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uart_node");
    ros::NodeHandle nh;

    sub_motor = nh.subscribe("/actuator/motor", 10, subMotorCallback);

    pub_encoder = nh.advertise<robot_msgs::encoder>("/sensor/encoder", 10);
    pub_ultrasonic = nh.advertise<robot_msgs::ultrasonic>("/sensor/ultrasonic", 10);

    timer_serialWrite = nh.createTimer(ros::Duration(0.001), serialWriteCallback);
    timer_serialRead = nh.createTimer(ros::Duration(0.001), serialReadCallback);

    ///inisialisasi serial dengan mikrokontroller
    try
    {
        serial.openDevice("/dev/ttyUSB-SERIAL", 115200);

        if(serial.isDeviceOpen())
        {
            ROS_INFO_STREAM("Serial Port initialized");
        }
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Unable to open port, try to recconect");
    }
    /////end of inisialisasi////////

    ros::spin();

    return 0;
}