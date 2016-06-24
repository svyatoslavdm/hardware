#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "sensor_msgs/JointState.h"
#include "schunk_ezn64_driver/functions.h"
#include "schunk_ezn64_driver/move.h"
#include "schunk_ezn64_driver/stop.h"
#include "schunk_ezn64_driver/open.h"
#include "schunk_ezn64_driver/close.h"

using namespace std;

Serial *objSerial;
Functions *objFunctions;

double grasping_force, sensor_data_0, sensor_data_1, sensor_data_2;
bool stopRead, stopClose;

ros::Publisher      ezn_state,
                    ezn_joints;
ros::Subscriber     ezn_forces;
ros::ServiceServer  movePosSS,
                    moveVelSS,
                    setTargetVelSS,
                    stopSS,
                    fastStopSS,
                    ackSS,
                    openSS,
                    closeSS;
                    
bool movePosSrv(schunk_ezn64_driver::move::Request &req, schunk_ezn64_driver::move::Response &res)
{
    res.ans = objFunctions->MovePos(req.val);
    if(res.ans)
        cout<<"MovePos ["<<req.val<<" mm]"<<endl;
    return true;
}

bool moveVelSrv(schunk_ezn64_driver::move::Request &req, schunk_ezn64_driver::move::Response &res)
{
    res.ans = objFunctions->MoveVel(req.val);
    if(res.ans)
        cout<<"MoveVel ["<<req.val<<" mm/s]"<<endl;
    return true;
}

bool setTargetVelSrv(schunk_ezn64_driver::move::Request &req, schunk_ezn64_driver::move::Response &res)
{
    res.ans = objFunctions->SetTargetVel(req.val);
    if(res.ans)
        cout<<"SetTargetVel ["<<req.val<<" mm/s]"<<endl;
    return true;
}

bool stopSrv(schunk_ezn64_driver::stop::Request &req, schunk_ezn64_driver::stop::Response &res)
{
    res.ans = objFunctions->Stop();
    if(res.ans)
        cout<<"Stop"<<endl;
    return true;
}

bool fastStopSrv(schunk_ezn64_driver::stop::Request &req, schunk_ezn64_driver::stop::Response &res)
{
    res.ans = objFunctions->FastStop();
    if(res.ans)
        cout<<"FastStop"<<endl;
    return true;
}

bool ackSrv(schunk_ezn64_driver::stop::Request &req, schunk_ezn64_driver::stop::Response &res)
{
    res.ans = objFunctions->Ack();
    if(res.ans)
        cout<<"Ack"<<endl;
    return true;
}

bool openSrv(schunk_ezn64_driver::open::Request &req, schunk_ezn64_driver::open::Response &res)
{
    res.ans = objFunctions->MoveVel(10.0);
    if(res.ans)
        cout<<"Opening"<<endl;
    return true;
}

bool closeSrv(schunk_ezn64_driver::close::Request &req, schunk_ezn64_driver::close::Response &res)
{
    res.ans = objFunctions->MoveVel(-5.0);
    if(res.ans)
        cout<<"Closing"<<endl;
        
    stopClose = false;
    ros::Time startClose = ros::Time::now();
     while(!stopClose)
     {
         if(req.grasping_force < sensor_data_0 || req.grasping_force < sensor_data_1 || req.grasping_force < sensor_data_2 || (ros::Time::now() - startClose).toSec() > 6.0)
         {
             cout<<"grasping_force < sensor_data"<<endl;
             stopClose = true;
         }        
         ros::Duration(0.05).sleep();
     }
    
    
    res.ans = objFunctions->Stop();
    if(res.ans)
        cout<<"Stop"<<endl;
        
    return true;
}

void serialRead() 
{
    cout<<"Reading start"<<endl;
    schunk_ezn64_driver::state msg;
    sensor_msgs::JointState joint_msg;

//    joint_msg.header.frame_id = "world";
    joint_msg.position.resize(3);
    joint_msg.velocity.resize(3);
    joint_msg.effort.resize(3);
    joint_msg.name.push_back("ezn_64/palm_joint_gripper_1");
    joint_msg.name.push_back("ezn_64/palm_joint_gripper_2");
    joint_msg.name.push_back("ezn_64/palm_joint_gripper_3");
    
    while(!stopRead)
    {
        if(objSerial->Read(msg))
        {
            if(msg.status != "" && msg.error != "")
            {
//                cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
//                cout<<"position :: "<<msg.position<<endl;
//                cout<<"speed    :: "<<msg.speed<<endl;
//                cout<<"current  :: "<<msg.current<<endl;
//                cout<<"status   :: "<<msg.status<<endl;
//                cout<<"error    :: "<<msg.error<<endl;
                if(msg.position < 0.01)
                {
                    stopClose = true;
                }                   
                ezn_state.publish(msg);
                
                joint_msg.header.stamp = ros::Time::now();;
                joint_msg.position[0] = msg.position/1000;
                joint_msg.position[1] = msg.position/1000;
                joint_msg.position[2] = msg.position/1000;
//                joint_msg.velocity[0] = msg.speed;
//                joint_msg.velocity[1] = msg.speed;
//                joint_msg.effort[0] = sensor_data_0;
//                joint_msg.effort[1] = sensor_data_1;
                ezn_joints.publish(joint_msg);
            }
        }  
        else
        {
            cout<<"Package not found. Check connection."<<endl;
            this_thread::sleep_for (chrono::milliseconds(10));  
        }       
    }    
    cout<<"Reading ended"<<endl;
}

void forcesCallback(const std_msgs::Float32MultiArray::ConstPtr& forces)
{
    sensor_data_0 = forces->data[0];
    sensor_data_1 = forces->data[1];
    sensor_data_2 = forces->data[2];
//    cout<<"data[0] = "<<forces->data[0]<<endl;
//    cout<<"data[1] = "<<forces->data[1]<<endl;
}

void sigint_handler(int sig) 
{
    cout<<"\nExiting..."<<endl;
    stopRead = true;
    objFunctions->Stop();
    sleep(1);
    objSerial->Disconnect();
    sleep(1);
    delete objSerial;
    delete objFunctions;
    ros::shutdown();  
}

int main( int argc, char **argv )
{
    ros::init(argc, argv, "ezn_64");
    ros::NodeHandle nh("~");
    signal(SIGINT, sigint_handler);

    double period;
    string device;

    nh.param("device", device, string("/dev/ttyUSB0"));
    nh.param("period", period, 0.1);   

    ezn_state = nh.advertise<schunk_ezn64_driver::state>("state", 1000);
    ezn_joints = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    ezn_forces = nh.subscribe("/schunk_ezn64_sensors/forces", 1000, forcesCallback);
    
    movePosSS = nh.advertiseService("move_pos", movePosSrv);  
    moveVelSS = nh.advertiseService("move_vel", moveVelSrv);   
    setTargetVelSS = nh.advertiseService("set_target_vel", setTargetVelSrv);  
    stopSS = nh.advertiseService("stop", stopSrv);  
    fastStopSS= nh.advertiseService("fast_stop", fastStopSrv);  
    ackSS= nh.advertiseService("ack", ackSrv);
    openSS= nh.advertiseService("open", openSrv);
    closeSS= nh.advertiseService("close", closeSrv);  
    
    ros::MultiThreadedSpinner spinner(2);
    ros::Duration(3.0).sleep();
   
   // Connect to device
   objSerial = new Serial(device);
   objFunctions = new Functions(objSerial);
   
   int fd = objSerial->Connect();
   
   if(fd < 0)
   {
        cout<<"Serial hasn't connected"<<endl;     
         return 0;  
   }
   else
   {
        cout<<"Serial has connected"<<endl;
        
        if(objFunctions->Reference())
            cout<<"Reference"<<endl;
        if(objFunctions->GetState(period))
            cout<<"GetState"<<endl;
        stopRead = false;
        thread t (serialRead);
        
        spinner.spin();
        ros::waitForShutdown();
        
        return 0;    
   }
}
