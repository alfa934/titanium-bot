#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <poll.h>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

typedef struct 
{
    int32_t motor_a;
    int32_t motor_b;
    int32_t dribbler;
    int32_t solenoid;
} bt_data ;


const char* dest = "F0:24:F9:F7:59:C2";
int sock;
struct pollfd fds[1];

uint8_t tx_buffer[64] = "ABC";
uint8_t rx_buffer[64];
uint8_t recv_buffer[64] = {0};
int recv_bytes = 0;

bt_data robot_tx;
bt_data robot_rx;

bool data_tx_ready = false;
bool connection_established = false;


ros::Timer timer;
ros::Subscriber sub;
ros::Time last_msg_time;
const double TIMEOUT = 1.0;


int setNonBlocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags == -1) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int Socket_Init()
{
    sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (sock < 0)
    {
        ROS_ERROR("Failed to create socket");
        return 0;
    }

    if (setNonBlocking(sock) < 0)
    {
        ROS_ERROR("Failed to set non-blocking mode");
        close(sock);
        return 0;
    }

    struct sockaddr_rc addr = {0};
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t)1;
    str2ba(dest, &addr.rc_bdaddr);

    ROS_INFO("Connecting to %s...", dest);
    int status = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    
    if (status < 0 && errno != EINPROGRESS)
    {
        ROS_ERROR("Connection failed: %s", strerror(errno));
        close(sock);
        return 0;
    }

    return 1;
}

void BluetoothConnect()
{
    if (!connection_established && (fds[0].revents & POLLOUT))
    {
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error)
        {
            ROS_ERROR("Connection failed: %s", error ? strerror(error) : "Unknown");
            return;
        }
        ROS_INFO("Connected successfully!");
        connection_established = true;
    }
}

void ReceiveData()
{
    if (fds[0].revents & POLLIN)
    {
        ssize_t received = read(sock, rx_buffer, sizeof(rx_buffer));
        
        if (received > 0)
        {
            for (int i = 0; i < received; i++)
            {
                if (recv_bytes < sizeof(recv_buffer))
                {
                    recv_buffer[recv_bytes++] = rx_buffer[i];
                }
                
                if (recv_bytes == sizeof(recv_buffer))
                {
                    if (recv_buffer[0] == 'A' && recv_buffer[1] == 'B' && recv_buffer[2] == 'C')
                    {
                        memcpy(&robot_rx.motor_a, recv_buffer + 3, 4);
                        memcpy(&robot_rx.motor_b, recv_buffer + 7, 4);
                        memcpy(&robot_rx.dribbler, recv_buffer + 11, 4);
                        memcpy(&robot_rx.solenoid, recv_buffer + 15, 4);

                        ROS_INFO("M_A,M_B,DRB,SOL: %d ; %d ; %d ; %d", robot_rx.motor_a,
                        robot_rx.motor_b, robot_rx.dribbler, robot_rx.solenoid);
                    }
                    else
                    {
                        ROS_WARN("Invalid header received");
                    }
                    recv_bytes = 0;
                }
            }
        }
        else if (received == 0)
        {
            ROS_ERROR("Connection closed by peer");
            return;
        }
        else if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            ROS_ERROR("Receive error: %s", strerror(errno));
            return;
        }
    }
}

void SendData()
{
    if (data_tx_ready && connection_established)
    {
        ssize_t data_sent = write(sock, tx_buffer, sizeof(tx_buffer));
        if (data_sent < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                ROS_ERROR("Send error: %s", strerror(errno));
                return;
            }
        } 
        else if (data_sent > 0)
        {
            data_tx_ready = false;
        }
    }
}

void sendCallback(const ros::TimerEvent&)
{
    if ((ros::Time::now() - last_msg_time).toSec() > TIMEOUT)
    {
        robot_tx.motor_a = 0;
        robot_tx.motor_b = 0;
        robot_tx.dribbler = 1; //--- 1 is off, 0 is on
        robot_tx.solenoid = 0;
        ROS_WARN_STREAM("Publisher timeout - resetting values");
    }
    
    memcpy(tx_buffer + 3, &robot_tx.motor_a, 4);
    memcpy(tx_buffer + 7, &robot_tx.motor_b, 4);
    memcpy(tx_buffer + 11, &robot_tx.dribbler, 4);
    memcpy(tx_buffer + 15, &robot_tx.solenoid, 4);

    data_tx_ready = true;
}

void subCallback(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    robot_tx.motor_a = msg->data[0];
    robot_tx.motor_b = msg->data[1];
    robot_tx.dribbler = msg->data[2];
    robot_tx.solenoid = msg->data[3];

    ROS_INFO("M_A,M_B,DRB,SOL: %d ; %d ; %d ; %d", robot_tx.motor_a,
    robot_tx.motor_b, robot_tx.dribbler, robot_tx.solenoid);

    last_msg_time = ros::Time::now();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_comm_node");
    ros::NodeHandle nh;
    
    if (!Socket_Init())
    {
        ROS_ERROR("Socket initialization failed");
        return 1;
    }

    last_msg_time = ros::Time::now();
    sub = nh.subscribe("/controller/pub", 10, subCallback);
    timer = nh.createTimer(ros::Duration(0.01), sendCallback);
    
    fds[0].fd = sock;
    fds[0].events = POLLOUT | POLLIN;
    
    ROS_INFO("Bluetooth Node started!");

    while (ros::ok())
    {
        int poll_return = poll(fds, 1, 100);
        
        if (poll_return < 0)
        {
            ROS_ERROR("Poll error: %s", strerror(errno));
            break;
        }
        
        if (poll_return > 0)
        {
            BluetoothConnect();
            
            SendData();

            // ReceiveData();
            
            if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                ROS_ERROR("Socket error (revents: 0x%X)", fds[0].revents);
                break;
            }
        }
        
        ros::spinOnce();
    }


    close(sock);
    ROS_INFO("Disconnected");
    return 0;
}