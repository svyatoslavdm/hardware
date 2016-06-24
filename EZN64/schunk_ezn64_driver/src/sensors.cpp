#include <arpa/inet.h>
#include <ros/ros.h>
#include <signal.h>
#include <thread>
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Empty.h"

#define ADDR_EZN            0xC0A805F3          // 192.168.5.243
#define UDPPORT             1025

#define FACTOR_1            0.015625                          // ID 802012    mV/Nmm
#define BASIC_GRADIENT_1    8.1175                            // mV/N
#define FACTOR_2            -0.008814                         // ID 802011    mV/Nmm
#define BASIC_GRADIENT_2    9.4573                            // mV/N
#define FACTOR_3            -0.010105                         // ID 802001    mV/Nmm
#define BASIC_GRADIENT_3    9.0627                            // mV/N

using namespace std;
    
bool connected = false;
bool wait = false;
int sockfd;
int num_sensors;

char buf[256];
double finger_distance;

struct sockaddr_in socket_pc, socket_device;
socklen_t slen = sizeof(socket_device);
ros::Publisher force_pub;

struct sensors
{
    const char * command;
    float factor;
    float zero;
};

vector <sensors> gripper_sensors;

// Set 0V on DO 0, DO 1 after driver starts
void set_output_on()    
{
    int res_len;
    if (sendto(sockfd, "#01D01\r", 7, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send command"); connected = false; return; }     
    else if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) != 4)
    { perror("Error receiving response"); connected = false; return; }
    
    ROS_INFO("Channel 0 set 0 V");
    ros::Duration(0.1).sleep();
    
    if (sendto(sockfd, "#01D11\r", 7, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send command"); connected = false; return; }     
    else if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) != 4)
    { perror("Error receiving response"); connected = false; return; }
    
    ROS_INFO("Channel 1 set 0 V");
}

// Set 24V on DO 0, DO 1 for 100 ms
void reset()
{    
    int res_len;
    if (sendto(sockfd, "#01D00\r", 7, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send reset command"); connected = false; return; }     
    else if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) != 4)
    { perror("Error receiving reset response"); connected = false; return; }
    
    ROS_INFO("Channel 0 set OFF (reset)");
    ros::Duration(0.1).sleep();
    
    if (sendto(sockfd, "#01D01\r", 7, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send reset command"); connected = false; return; }     
    else if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) != 4)
    { perror("Error receiving reset response"); connected = false; return; }
    
    ROS_INFO("Channel 0 set ON");
    ros::Duration(0.1).sleep();
        
    if (sendto(sockfd, "#01D10\r", 7, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send reset command"); connected = false; return; }     
    else if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) != 4)
    { perror("Error receiving reset response"); connected = false; return; }
    
    ROS_INFO("Channel 1 set OFF (reset)");
    ros::Duration(0.1).sleep();
        
    if (sendto(sockfd, "#01D11\r", 7, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send reset command"); connected = false; return; }     
    else if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) != 4)
    { perror("Error receiving reset response"); connected = false; return; }
    
    ROS_INFO("Channel 1 set ON");    
}

float calc_data(float volts, sensors sensor)
{    
    return ((volts - sensor.zero) * 1000 / sensor.factor);       
}

float read_data(sensors sensor)
{    
    int res_len;
    if (sendto(sockfd, sensor.command, 5, 0, (struct sockaddr*)&socket_device, (int)slen) < 0)
    { perror("Failed to send message"); connected = false; return 0; }    
    
	if ((res_len = recvfrom(sockfd, buf, 256, 0, (struct sockaddr *)&socket_device, &slen)) < 0)
    { perror("Failed to receive message"); connected = false; return 0; }
    else
    {
        if (res_len == 12)
        {
            char res[res_len - 1];
            for (int i = 1; i < res_len; i++)
                res[i - 1] = buf[i];
            return atof(res);       
        }
        else 
        { perror("Error message"); return 0;    } 
    }
}

bool resetSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
    if (connected)
    {
        wait = true;
        printf("Reseting FMS...\n");
        reset();
        ros::Duration(0.5).sleep();
		for (int i = 0; i < (int)gripper_sensors.size(); i++)
		{
			gripper_sensors[i].zero = read_data(gripper_sensors[i]);
			ros::Duration(0.5).sleep();
		}
        wait = false;
    }
    else
    {
        perror("Failed to connect with sensors");
        return false;
    } 
    return true;
}

void sigint_handler(int sig) 
{
    printf("Exiting\n");
    close(sockfd);
    printf("Socket closed\n");
    ros::shutdown();
}

void ascii_thread()
{
    printf("Started sending ASCII commands\n");
    
    std_msgs::Float32MultiArray msg;
    msg.data.resize(num_sensors);
       
    while(connected)
    {
        if(!wait)
        {
			for (int i = 0; i < num_sensors; i++)
			{
				msg.data[i] = calc_data(read_data(gripper_sensors[i]), gripper_sensors[i]);
			}
            force_pub.publish(msg);	 
        }
    }
    printf("ASCII command thread finished\n");
}

int main( int argc, char **argv )
{
    ros::init(argc, argv, "ezn_64_sensors");
    ros::NodeHandle nh("~");
    signal(SIGINT, sigint_handler);
    
    nh.param("distance", finger_distance, 20.0);

	num_sensors = 3;
	gripper_sensors.resize(3);

    force_pub = nh.advertise<std_msgs::Float32MultiArray>("forces", 1000);
    ros::ServiceServer reset_srv = nh.advertiseService("reset_sensors", resetSrv);
    
    //Open connection    
    if ((sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0) //IPPROTO_UDP
    {
        printf("Failed to create socket");
        return 0;
    }
            
    memset((char *) &socket_pc, 0, sizeof(socket_pc));
    socket_pc.sin_family = AF_INET;
    socket_pc.sin_port = htons(UDPPORT);
    socket_pc.sin_addr.s_addr = htonl(INADDR_ANY);
    
    socket_device.sin_family = AF_INET;
    socket_device.sin_port = htons(UDPPORT);
    socket_device.sin_addr.s_addr = htonl(ADDR_EZN);
    
    if (bind(sockfd, (struct sockaddr *)&socket_pc, sizeof(socket_pc)) < 0)
    {
        perror("Failed to bind socket");
        close(sockfd);
        return 0;
    }
    connected = true;

	gripper_sensors[0].factor = FACTOR_1 * finger_distance + BASIC_GRADIENT_1;
    gripper_sensors[0].command = "#014\r";
    gripper_sensors[0].zero = 0.0;
    
    gripper_sensors[1].factor = FACTOR_2 * finger_distance + BASIC_GRADIENT_2;
    gripper_sensors[1].command = "#015\r";
    gripper_sensors[1].zero = 0.0;

    gripper_sensors[2].factor = FACTOR_3 * finger_distance + BASIC_GRADIENT_3;
    gripper_sensors[2].command = "#013\r";
    gripper_sensors[2].zero = 0.0;
    
    //Reset force measurement system
    set_output_on();
    ros::Duration(0.5).sleep();
    printf("Channels DO 0, DO 1 set 0V\n");
    reset();
    
    ros::Duration(0.5).sleep();
    printf("Starting FMS zero adjustment\n");
	gripper_sensors[0].zero = read_data(gripper_sensors[0]);
    ros::Duration(0.5).sleep();
	gripper_sensors[1].zero = read_data(gripper_sensors[1]);
    ros::Duration(0.5).sleep();
	gripper_sensors[2].zero = read_data(gripper_sensors[2]);
    printf("Zero adjustment complete\n");
        
    //Start thread
    std::thread read_th;
    read_th = std::thread(ascii_thread);
    
    ros::spin();
    ros::shutdown();
    
    return 0;
}
