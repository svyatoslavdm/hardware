/*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"
#include "wsg_50/functions_serial.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"
#include "wsg_50_common/Cmd.h"
#include "wsg_50_common/Limits.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0


ros::Publisher g_pub_state, g_pub_joint, g_pub_moving;
bool g_ismoving = false;
bool g_thread = false;   
finger_info info_fingers[2];
bool use_fmf;

bool moveSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) )
	{
  		ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
		res.error = move(req.width, req.speed, false);
	}
	else if (req.width < 0.0 || req.width > 110.0)
	{
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}
	else
	{
	        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		res.error = move(req.width, req.speed, false);
	}

	ROS_INFO("Target position reached.");
  	return true;
}

bool graspSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) )
	{
        ROS_INFO("Grasping object at %f with %f mm/s.", req.width, req.speed);
		res.error = grasp(req.width, req.speed);
	}
	else if (req.width < 0.0 || req.width > 110.0)
	{
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}
	else
	{
        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = grasp(req.width, req.speed);
	}

	ROS_INFO("Object grasped correctly.");	
  	return true;
}

bool releaseSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) )
	{
  		ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
		res.error = release(req.width, req.speed);
	}
	else if (req.width < 0.0 || req.width > 110.0)
	{
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}
	else
	{
        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = release(req.width, req.speed);
	}
	ROS_INFO("Object released correctly.");
  	return true;
}

bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_INFO("Homing...");
	homing();
	ROS_INFO("Home position reached.");
	return true;
}

bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_WARN("Stop!");
	stop();
	ROS_WARN("Stopped.");
	return true;
}

bool setAccSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setAcceleration(req.val);
	return true;
}

bool setForceSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setGraspingForceLimit(req.val);
	return true;
}

bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ack_fault();
	return true;
}

bool moveSrv_no_waiting(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
    // Send command to gripper without waiting for a response
    if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) )
    {
  		ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
  		res.error = move_serial(req.width, req.speed, false);
	}
	else if (req.width < 0.0 || req.width > 110.0)
	{
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}
	else
	{
        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		res.error = move_serial(req.width, req.speed, false);
	}
  	return true;
}

bool homingSrv_no_waiting(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_INFO("Homing");
	homing_serial();
	return true;
}

bool stopSrv_no_waiting(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_WARN("Stopping gripper");
	stop_serial();
	return true;
}

bool graspSrv_no_waiting(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) )
	{
        ROS_INFO("Grasping object at %f with %f mm/s.", req.width, req.speed);
		res.error = grasp_serial(req.width, req.speed);
	}
	else if (req.width < 0.0 || req.width > 110.0)
	{
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}
	else
	{
        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = grasp_serial(req.width, req.speed);
	}

  	return true;
}

bool releaseSrv_no_waiting(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) )
	{
  		ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
		res.error = release_serial(req.width, req.speed);
	}
	else if (req.width < 0.0 || req.width > 110.0)
	{
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}
	else
	{
        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = release_serial(req.width, req.speed);
	}
	
  	return true;
}


bool setAccSrv_no_waiting(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setAcceleration_serial(req.val);
	return true;
}

bool setForceSrv_no_waiting(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setGraspingForceLimit_serial(req.val);
	return true;
}

bool setLimitsSrv(wsg_50_common::Limits::Request &req, wsg_50_common::Limits::Response &res)
{
    if ((req.limit_minus > req.limit_plus) || (req.limit_plus > 110.0) || (req.limit_minus < 0.0))
    {
        ROS_ERROR("Invalid soft limits");
        return false;
    }
	setSoftLimits_serial(req.limit_minus, req.limit_plus);
	return true;
}

bool clearLimitsSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	clearSoftLimits_serial();
	return true;
}

bool getLimitsSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	getSoftLimits_serial();
	return true;
}

/** \brief Reads gripper responses in auto_update mode. The gripper pushes state messages in regular intervals. */
void read_thread(int interval_ms)
{
    ROS_INFO("Thread started");

    status_t status;
    int res;
    bool pub_state = false;

    // Prepare messages
    wsg_50_common::Status status_msg;
    status_msg.status = "";

    sensor_msgs::JointState joint_states;
//    joint_states.header.frame_id = "wsg_50/palm_link";
    joint_states.name.push_back("wsg_50/palm_joint_gripper_left");
    joint_states.name.push_back("wsg_50/palm_joint_gripper_right");
    joint_states.position.resize(2);
    joint_states.velocity.resize(2);
    joint_states.effort.resize(2);

    // Request automatic updates (error checking is done below)
    getOpening(interval_ms);
    getSpeed(interval_ms);
    getForce(interval_ms);

    msg_t msg; msg.id = 0; 
               msg.data = 0; 
               msg.len = 0;
    int cnt[3] = {0,0,0};
    auto time_start = std::chrono::system_clock::now();

    // Prepare FMF data reading
    std::vector<float> finger_data;     finger_data.push_back(0.0);
                                        finger_data.push_back(0.0);
    int index = 0;
    bool waiting_fmf = false;
    unsigned char vResult[4];
    
    while (g_thread) 
    {
        // Receive gripper response
        msg_free(&msg);
        res = msg_receive( &msg );
        if (res < 0 )   //|| msg.len < 2) 
        {
            ROS_ERROR("Gripper response failure");
            continue;
        }

        float val = 0.0;
        
        status = cmd_get_response_status(msg.data);

        // Decode float for opening/speed/force
        if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) 
        {
            if (status != E_SUCCESS) 
            {
                ROS_ERROR("Gripper response failure for opening/speed/force\n");
                continue;
            }
            val = convert(&msg.data[2]);
        }
        
        // Request force measurement
//        if (use_fmf && !waiting_fmf)
//        {
//            waiting_fmf = true;
//            getFingerData_serial(index);
//        }        
        if ((info_fingers[index].type == 1) && !waiting_fmf)
        {
            waiting_fmf = true;
            getFingerData(index, true);
        }
 
        // Handle response types
        int motion = -1;
          
        switch (msg.id) 
        {
        /*** Opening ***/
        case 0x43:
            status_msg.width = val;
            pub_state = true;
            cnt[0]++;
            break;

        /*** Speed ***/
        case 0x44:
            status_msg.speed = val;
            cnt[1]++;
            break;

        /*** Force ***/
        case 0x45:
            status_msg.force = val;
            cnt[2]++;
            break;
        
        /*** Home ***/
        case 0x20:
            // Homing command; nothing to do
            break;
        case 0x22:
            // Stop command; nothing to do
            break;
              
        /*** Move ***/   
        case 0x21:
        // Move commands are sent from outside this thread   
        case 0x25:
        // Grasping object
        case 0x26:
        // Releasing object
            if (status == E_SUCCESS) 
            {
                status_msg.status = std::string(status_to_str(status));
                ROS_INFO("Position reached");
                motion = 0;
            } 
            else if (status == E_AXIS_BLOCKED) 
            {
                status_msg.status = std::string(status_to_str(status));
                ROS_INFO("Axis blocked");
                motion = 0;
            } 
            else if (status == E_CMD_PENDING) 
            {
                status_msg.status = std::string(status_to_str(status));
                ROS_INFO("Movement started");
                motion = 1;
            } 
            else if (status == E_ALREADY_RUNNING) 
            {
                status_msg.status = std::string(status_to_str(status));
                ROS_INFO("Movement error: already running");
            } 
            else if (status == E_CMD_ABORTED) 
            {
                status_msg.status = std::string(status_to_str(status));
                ROS_INFO("Movement aborted");
                motion = 0;
            } 
            else 
            {
                status_msg.status = std::string(status_to_str(status));
                ROS_INFO("Movement error");
                motion = 0;
            }
            break;
            
        /*** Finger Data ***/
        case 0x63:
            if (status == E_SUCCESS)
            {
                vResult[0] = msg.data[2];
                vResult[1] = msg.data[3];
                vResult[2] = msg.data[4];
                vResult[3] = msg.data[5];
                finger_data[index] = convert(vResult);
            }
            else
                finger_data[index] = 0.0;
            waiting_fmf = false;
            index = abs(index - 1);            
            break;
            
        /*** Soft Limits ***/
        case 0x34:
            if (status == E_SUCCESS)    ROS_INFO("New Soft Limits");
            break;
            
        case 0x35:
            if (status == E_SUCCESS && msg.len == 10)
            {   
                unsigned char limit_minus[4];
                unsigned char limit_plus[4];
	            limit_minus[0] = msg.data[2];
	            limit_minus[1] = msg.data[3];
	            limit_minus[2] = msg.data[4];
	            limit_minus[3] = msg.data[5];
	            limit_plus[0] = msg.data[6];
	            limit_plus[1] = msg.data[7];
	            limit_plus[2] = msg.data[8];
	            limit_plus[3] = msg.data[9];
	            
                ROS_INFO("Soft limits: \nLIMIT MINUS  %f\nLIMIT PLUS   %f\n", convert(limit_minus), convert(limit_plus));
            }
            else    ROS_INFO("Failed to read soft limits, len %d", msg.len);
            break;
            
        case 0x36:
            if (status == E_SUCCESS)    ROS_INFO("Soft Limits Cleared");
            break;
               
        default:
            ROS_INFO("Received response 0x%02x (%2dB)\n", msg.id, msg.len);
        }
        
        // ***** PUBLISH motion message
        if (motion == 0 || motion == 1) 
        {
            std_msgs::Bool moving_msg;
            moving_msg.data = motion;
            g_pub_moving.publish(moving_msg);
            g_ismoving = motion;
        }

        // ***** PUBLISH state message & joint message
        if (pub_state) 
        {
            pub_state = false;
            
            if (info_fingers[0].type == 1)
	            status_msg.force_finger0 = finger_data[0];
            else
                status_msg.force_finger0 = status_msg.force/2;
            
            if (info_fingers[1].type == 1)
                status_msg.force_finger1 = finger_data[1];
            else
                status_msg.force_finger1 = status_msg.force/2;
                
            g_pub_state.publish(status_msg);

            joint_states.header.stamp = ros::Time::now();;
            joint_states.position[0] = -status_msg.width/2000.0;
            joint_states.position[1] = status_msg.width/2000.0;
            joint_states.velocity[0] = status_msg.speed/1000.0;
            joint_states.velocity[1] = status_msg.speed/1000.0;
            joint_states.effort[0] = status_msg.force_finger0;
	        joint_states.effort[1] = status_msg.force_finger1;

            g_pub_joint.publish(joint_states);
        }

        // Check # of received messages regularly
        
        double rate_exp = 1000.0 / (double)interval_ms;
        std::string names[3] = { "opening", "speed", "force" };

        std::chrono::duration<float> t = std::chrono::system_clock::now() - time_start;
        double t_ = t.count();
        if (t_ > 5.0) 
        {
            time_start = std::chrono::system_clock::now();
            //printf("Infos for %5.1fHz, %5.1fHz, %5.1fHz\n", (double)cnt[0]/t_, (double)cnt[1]/t_, (double)cnt[2]/t_);

            std::string info = "Rates for ";
            for (int i=0; i<3; i++) 
            {
                double rate_is = (double)cnt[i]/t_;
                info += names[i] + ": " + std::to_string((int)rate_is) + "Hz, ";
                if (rate_is == 0.0)
                    ROS_ERROR("Did not receive data for %s", names[i].c_str());
            }
            ROS_DEBUG_STREAM((info + " expected: " + std::to_string((int)rate_exp) + "Hz").c_str());
            cnt[0] = 0; cnt[1] = 0; cnt[2] = 0;
        }
    }

    // Disable automatic updates
    getOpening(0);
    getSpeed(0);
    getForce(0);

    ROS_INFO("Thread ended");
}

void sigint_handler(int sig) 
{
    ROS_WARN("Exiting...");
    g_thread = false;
    stop();
    sleep(5);
    cmd_disconnect();
    ros::shutdown();
}



int main( int argc, char **argv )
{
   ros::init(argc, argv, "wsg_50");
   ros::NodeHandle nh("~");
   signal(SIGINT, sigint_handler);
   
   double rate, grasping_force;
   std::string device;
   int bitrate;
   
   nh.param("read_rate", rate, 30.0);
   nh.param("grasping_force", grasping_force, 80.0);
   nh.param("device", device, std::string("/dev/ttyUSB3"));
   nh.param("bitrate", bitrate, 38400);
   nh.param("use_fmf", use_fmf, true);
   
   // Connect to device
   int res_con;
   
   ROS_INFO("Connecting to %s: (rs232) at %d ...", device.c_str(), bitrate);
   res_con = cmd_connect_serial(device.c_str(), bitrate);

   if (res_con == 0 ) {
        ROS_INFO("Gripper connection stablished");

		// Services
        ros::ServiceServer moveSS, graspSS, releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS, setForceSS;
        
        ros::ServiceServer move_no_waitingSS, grasp_no_waitingSS, release_no_waitingSS, 
                           homing_no_waitingSS, stop_no_waitingSS, setAcc_no_waitingSS, 
                           setForce_no_waitingSS, setLimitsSS, clearLimitsSS, getLimitsSS;

        moveSS = nh.advertiseService("move", moveSrv);
        graspSS = nh.advertiseService("grasp", graspSrv);
        releaseSS = nh.advertiseService("release", releaseSrv);
        homingSS = nh.advertiseService("homing", homingSrv);
        stopSS = nh.advertiseService("stop", stopSrv);
        ackSS = nh.advertiseService("ack", ackSrv);

        setAccSS = nh.advertiseService("set_acceleration", setAccSrv);
        setForceSS = nh.advertiseService("set_force", setForceSrv);
        
        move_no_waitingSS = nh.advertiseService("move_no_waiting", moveSrv_no_waiting);
        grasp_no_waitingSS = nh.advertiseService("grasp_no_waiting", graspSrv_no_waiting);
        release_no_waitingSS = nh.advertiseService("release_no_waiting", releaseSrv_no_waiting);
        homing_no_waitingSS = nh.advertiseService("homing_no_waiting", homingSrv_no_waiting);
        stop_no_waitingSS = nh.advertiseService("stop_no_waiting", stopSrv_no_waiting);
        setAcc_no_waitingSS = nh.advertiseService("set_acceleration_no_waiting", setAccSrv_no_waiting);
        setForce_no_waitingSS = nh.advertiseService("set_force_no_waiting", setForceSrv_no_waiting);
        
        setLimitsSS = nh.advertiseService("set_soft_limits", setLimitsSrv);
        clearLimitsSS = nh.advertiseService("clear_soft_limits", clearLimitsSrv);
        getLimitsSS = nh.advertiseService("view_soft_limits", getLimitsSrv);
              
		// Publisher
		g_pub_state = nh.advertise<wsg_50_common::Status>("status", 1000);
		g_pub_joint = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        g_pub_moving = nh.advertise<std_msgs::Bool>("moving", 10);

		ROS_INFO("Ready to use, homing now...");
		homing();

		if (grasping_force > 0.0) {
			ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
			setGraspingForceLimit(grasping_force);
		}
		
		if (use_fmf)
		{
		    ROS_INFO("Force measure fingers data reading: ON");
		    ROS_INFO("Reading finger info (type 0 - generic, 1 - WSG-FMF) ...");
		    getFingerInfo(0, info_fingers[0]);
		    getFingerInfo(1, info_fingers[1]);
		    ROS_INFO("Finger 0 type %d: data size %d bytes", info_fingers[0].type, info_fingers[0].data_size);
		    ROS_INFO("Finger 1 type %d: data size %d bytes", info_fingers[1].type, info_fingers[1].data_size); 
		    
            if (info_fingers[0].type != info_fingers[1].type)
                info_fingers[0].type = info_fingers[1].type = 1;
		}
		else
		{
    		ROS_INFO("Force measure fingers data reading: OFF");
		    info_fingers[0].type = info_fingers[1].type = 0;
		}
            
        ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
                
        std::thread th;
        g_thread = true;
        th = std::thread(read_thread, (int)(1000.0/rate));
        
        ros::spin();

	} else {
        ROS_ERROR("Unable to connect, please check the port and address used.");
	}

	return 0;

}
