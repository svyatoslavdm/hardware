#include "ros/ros.h"

//Services including
#include "kawasaki_net/kaw_net_move.h"
#include "kawasaki_net/kaw_net_start.h"
#include "kawasaki_net/kaw_net_stop.h"
#include "kawasaki_net/kaw_net_getpos.h"
#include "kawasaki_net/kaw_net_speed.h"
#include "kawasaki_net/kaw_net_accur.h"
#include "kawasaki_net/kaw_net_acdec.h"
#include "kawasaki_net/kaw_net_exec.h"

ros::ServiceClient client_net_start;
ros::ServiceClient client_net_stop;
ros::ServiceClient client_net_move;
ros::ServiceClient client_net_getpos;
ros::ServiceClient client_net_speed;
ros::ServiceClient client_net_accur;
ros::ServiceClient client_net_acdec;
ros::ServiceClient client_net_exec;

kawasaki_net::kaw_net_start srv_net_start;
kawasaki_net::kaw_net_stop srv_net_stop;
kawasaki_net::kaw_net_move srv_net_move;
kawasaki_net::kaw_net_getpos srv_net_getpos;
kawasaki_net::kaw_net_speed srv_net_speed;
kawasaki_net::kaw_net_accur srv_net_accur;
kawasaki_net::kaw_net_acdec srv_net_acdec;
kawasaki_net::kaw_net_exec srv_net_exec;

using namespace std;

int ExecCommand (string command)
{
    srv_net_exec.request.command = command;
    client_net_exec.call(srv_net_exec);
    
    return srv_net_exec.response.Result;
}
  
int LMove (vector<float> pos)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 0;
    srv_net_move.request.speed = 0.0; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 0;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int LMoveT (vector<float> pos, float speed)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 0;
    srv_net_move.request.speed = speed; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 1;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int LMoveS (vector<float> pos, float speed)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 0;
    srv_net_move.request.speed = speed; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 0;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int JMove (vector<float> pos)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 1;
    srv_net_move.request.speed = 0.0; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 0;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int JMoveT (vector<float> pos, float speed)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 1;
    srv_net_move.request.speed = speed; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 1;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int JMoveS (vector<float> pos, float speed)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 1;
    srv_net_move.request.speed = speed; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 0;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int JMoveDecT (vector<float> pos, float speed)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 2;
    srv_net_move.request.speed = speed; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 1;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int JMoveDecS (vector<float> pos, float speed)
{
    srv_net_move.request.data = pos;
    srv_net_move.request.motiontype = 2;
    srv_net_move.request.speed = speed; 
    srv_net_move.request.angspeed = 0.0;
    srv_net_move.request.time = 0;
    
    client_net_move.call(srv_net_move);
    
    return srv_net_move.response.Result;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveclient");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);
  
  client_net_start = n.serviceClient<kawasaki_net::kaw_net_start>("kaw_net_start");
  client_net_stop = n.serviceClient<kawasaki_net::kaw_net_stop>("kaw_net_stop");
  client_net_move = n.serviceClient<kawasaki_net::kaw_net_move>("kaw_net_move");
  client_net_getpos = n.serviceClient<kawasaki_net::kaw_net_getpos>("kaw_net_getpos");
  client_net_speed = n.serviceClient<kawasaki_net::kaw_net_speed>("kaw_net_speed");
  client_net_accur = n.serviceClient<kawasaki_net::kaw_net_accur>("kaw_net_accur");
  client_net_acdec = n.serviceClient<kawasaki_net::kaw_net_acdec>("kaw_net_acdec");
  client_net_exec = n.serviceClient<kawasaki_net::kaw_net_exec>("kaw_net_exec");

  srv_net_start.request.Address = "192.168.5.242";
  
  if (client_net_start.call(srv_net_start))
  {
      if (srv_net_start.response.Result) 
      {
	  ROS_INFO ("kaw_net_start failed with error %d.", srv_net_start.response.Result);
	  return -1;
      }
  }
  else
  {
      ROS_INFO ("Failed to call service kaw_net_start.");
      return -1;
  }
  
  ExecCommand("CP ON");
  
  vector<float> pos(6);
  int res;
  

  pos[0] = 30;
  pos[1] = 40;
  pos[2] = -80;
  pos[3] = 20; 
  pos[4] = 30; 
  pos[5] = 0;
  res = JMoveT(pos, 5);
  ROS_INFO ("JMoveT res = %d", res);
  
/*  pos[0] = 35;
  pos[1] = 45;
  pos[2] = -85;
  pos[3] = 25; 
  pos[4] = 35; 
  pos[5] = 5;*/
  pos[0] = -200;
  pos[1] = 800;
  pos[2] = 400;
  pos[3] = 90; 
  pos[4] = 90; 
  pos[5] = -90;
	res = LMoveT(pos, 5);
  	while (ros::ok())
	{
		 	  
			pos[0] -= 10;
			res = LMoveT(pos, 0.11);
			ROS_INFO ("JMoveDecT res = %d", res);
			loop_rate.sleep();
	}
  	client_net_stop.call(srv_net_stop);
 
  	return 0;
}
