#include "ros/ros.h"
#include "stdio.h"
#include "sys/socket.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "boost/algorithm/string.hpp"
#include "boost/lexical_cast.hpp"

//Services including
#include "kawasaki_net/kaw_net_move.h"
#include "kawasaki_net/kaw_net_start.h"
#include "kawasaki_net/kaw_net_stop.h"
#include "kawasaki_net/kaw_net_getpos.h"
#include "kawasaki_net/kaw_net_speed.h"
#include "kawasaki_net/kaw_net_accur.h"
#include "kawasaki_net/kaw_net_acdec.h"
#include "kawasaki_net/kaw_net_exec.h"

using namespace std;
using namespace boost;
using boost::lexical_cast;
using boost::bad_lexical_cast;
    
int sClient;
struct sockaddr_in addr;
bool bConnecting;


bool SendMessage (string vcMessage)
{
  int ret = send(sClient, vcMessage.c_str(), vcMessage.length(), 0);
  if (ret < 0)
    return false;
  else 
  {
    //ROS_INFO ("Sending message: %s, sended bytes: %d", vcMessage.c_str(), ret);
    //printf ("Sending message: %s, sended bytes: %d\n", vcMessage.c_str(), ret);
    //printf ("Sending message: <<<%s>>>\n", vcMessage.c_str());
    return true;
  }
}

string ReceiveMessage ()
{
  string recvMessage;
  char cBuffer[513];
  
  memset(cBuffer, 0, sizeof(cBuffer));
  ssize_t ret = recv (sClient, &cBuffer, 512, 0);
  if (ret < 0)
  {
    recvMessage = "!!!ERROR!!!";
    ROS_INFO ("Receiving error.");
  }
  else
  {
    recvMessage = cBuffer;
    //ROS_INFO ("Receiving message: %s, received bytes: %d", recvMessage.c_str(), ret);
  }
  return recvMessage;
}

int8_t CheckReply (string vcReply)
{
    try
    {
	//int16_t ret = lexical_cast<int16_t>(vcReply);
	//printf ("<<<%s>>> casted to %d\n", vcReply.c_str(), ret);
	//return ret;
	return lexical_cast<int16_t>(vcReply);
    }
    catch (bad_lexical_cast &)
    {
	return -127;
    }
/*    
  if (vcReply == "0") return 0; //success
  if (vcReply == "1") return 1; //error( Target point not in motion range ).
  if (vcReply == "2") return 2; //error( Motion disallow by collision avoidance system ).
  if (vcReply == "3") return 3; //error( Velocity too small ).

  return -1; //return uncknown error
*/
}

string ExecCommand (string vcCommand)
{
  if (SendMessage(vcCommand))
    return ReceiveMessage();
  else 
    return "ERROR";
}

bool send_net_start(kawasaki_net::kaw_net_start::Request &req,
	       kawasaki_net::kaw_net_start::Response &res)
{
  if (bConnecting) 
  {
    ROS_INFO ("Socket is already started.");
    res.Result = 100;
    return true;
  }
  sClient = socket(AF_INET, SOCK_STREAM, 0);
  if (sClient < 0)
  {
    ROS_INFO ("Socket creating error.");
    res.Result = 1;
  }
  else
  {
    in_addr address;
    if (inet_aton (req.Address.c_str(), &address))
    {
      addr.sin_addr = address;
      addr.sin_family = AF_INET;
      addr.sin_port = htons(9001);
      sClient = socket(AF_INET, SOCK_STREAM, 0);
      if (sClient < 0) 
      {
	ROS_INFO ("Socket creating error.");
	res.Result = 2;
      } 
      else
      {
	//ROS_INFO ("Socket created.");
	if (connect(sClient, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
	  ROS_INFO ("Socket connecting error.");
	  res.Result = 3;
	}
	else
	{
	  if (!SendMessage("ping")) 
	  {
	    res.Result = 4;
	    ROS_INFO ("Can't send message to socket.");
	  }
	  else
	  {
	    ReceiveMessage();
	    printf ("Connection established.\n");
	    res.Result = 0;
	    bConnecting = true;
	  }
	}
      }
    }
  }
  return true;
}

bool send_net_stop(kawasaki_net::kaw_net_stop::Request &req,
	       kawasaki_net::kaw_net_stop::Response &res)
{
  if (!bConnecting)
  {
    ROS_INFO ("The connection can't be established.");
    res.Result = 100;
    return true;
  }

  close(sClient);
  //ROS_INFO("Connection closed.");
  printf ("Connection closed.\n");
  bConnecting = false;
  res.Result = 0;
  return true;
}

bool send_move(kawasaki_net::kaw_net_move::Request &req,
	       kawasaki_net::kaw_net_move::Response &res)
{
  string vcCommand;
  char pcCommand[512];

  switch (req.motiontype)
  {
    case 0 : vcCommand = "LMOVE"; break;
    case 1 : vcCommand = "JMOVE"; break;
    case 2 : vcCommand = "JMOVEDEC"; break;
    default : vcCommand = "ERROR";
  }
  if (req.speed > 0.0) vcCommand = vcCommand + (req.time ? "T" : "S");
  if (req.speed <= 0.0)
    sprintf (pcCommand, "%s %.2f %.2f %.2f %.3f %.3f %.3f", 
	     vcCommand.c_str(), req.data[0], req.data[1], req.data[2], req.data[3], req.data[4], req.data[5]); 
  else if (req.angspeed <= 0.0)
	  sprintf (pcCommand, "%s %.3f %.2f %.2f %.2f %.3f %.3f %.3f", 
		   vcCommand.c_str(), req.speed, req.data[0], req.data[1], req.data[2], req.data[3], req.data[4], req.data[5]); 
	  else  
	    	  sprintf (pcCommand, "%s %.3f %.3f %.2f %.2f %.2f %.3f %.3f %.3f", 
		   vcCommand.c_str(), req.speed, req.angspeed, req.data[0], req.data[1], req.data[2], req.data[3], req.data[4], req.data[5]); 
  res.Result = CheckReply (ExecCommand (pcCommand));
  //printf ("Result: %d\n", res.Result);
  return true;
}

bool send_getpos(kawasaki_net::kaw_net_getpos::Request &req,
	       kawasaki_net::kaw_net_getpos::Response &res)
{
    vector<float> locdata(7);
    vector< string > vsSplitVec;
  
    string cmdStr = req.dest ? (req.joints ? "DESTJ" : "DEST") : (req.joints ? "HEREJ" : "HERE");
    string rplStr = ExecCommand (cmdStr);
  
    trim(rplStr);
    split (vsSplitVec, rplStr, is_any_of(" ,"), token_compress_off);
 
    if (vsSplitVec.size() != 6)
    {
	locdata[6] = 0;
	return true;
    }
    else locdata[6] = 6;
  

    try
    {
	for (uint8_t index = 0; index < vsSplitVec.size(); index++) 
	    locdata[index] = lexical_cast<float>(vsSplitVec[index]);
    
	//for (uint8_t index = 0; index < vsSplitVec.size(); index++) 
	    //ROS_INFO ("%f", locdata[index]);
    }

    catch (bad_lexical_cast &)
    {
	locdata[6] = 0;
	return true;
    }
    res.data = locdata;
  
    return true;
}

bool send_speed(kawasaki_net::kaw_net_speed::Request &req,
	       kawasaki_net::kaw_net_speed::Response &res)
{
    string cmdStr = "SPEED ";
    cmdStr = cmdStr + lexical_cast<string>(req.speed) + " " + req.suffix + (req.always ? " ALWAYS" : ""); //Если нет суффикса, лишний пробел. Надо бы убрать.
    
    res.Result = CheckReply (ExecCommand(cmdStr));
  
    return true;
}

bool send_accur(kawasaki_net::kaw_net_accur::Request &req,
	       kawasaki_net::kaw_net_accur::Response &res)
{
    string cmdStr = "ACCURACY ";
    cmdStr = cmdStr + lexical_cast<string>(req.acc) + (req.always ? " ALWAYS" : "");
    
    res.Result = CheckReply (ExecCommand(cmdStr));
    
    return true;
}

bool send_acdec(kawasaki_net::kaw_net_acdec::Request &req,
	       kawasaki_net::kaw_net_acdec::Response &res)
{
    string cmdStr;
    cmdStr = (req.accel ? "ACCEL " : "DECEL ");
    cmdStr = cmdStr + lexical_cast<string>(req.acdec) + (req.always ? " ALWAYS" : "");
  
    res.Result = CheckReply (ExecCommand(cmdStr));
    
    return true;
}

bool send_exec(kawasaki_net::kaw_net_exec::Request &req,
	       kawasaki_net::kaw_net_exec::Response &res)
{
    res.Result = CheckReply (ExecCommand(req.command));
    
    return true;
}
/*
bool send_(kawasaki_net::kaw_net_::Request &req,
	       kawasaki_net::kaw_net_::Response &res)
{
}
*/

int main(int argc, char **argv)
{
	string ServerName = "kaw_net_server";
	if (argc > 1)
	{
		ServerName = ServerName + "_";
		ServerName = ServerName + argv[1];
	}
  ros::init(argc, argv, ServerName);
  ros::NodeHandle n;
  
  printf ("=======================================================================\n");
  printf ("Kawasaki services starting for %s...\n\n", ServerName.c_str());
  
  ros::ServiceServer ser_net_start = n.advertiseService(ServerName + "/kaw_net_start", send_net_start);
  printf ("Net_start service started.\n");
  ros::ServiceServer ser_net_stop = n.advertiseService(ServerName + "/kaw_net_stop", send_net_stop);
  printf ("Net_stop service started.\n");
  ros::ServiceServer ser_move = n.advertiseService(ServerName + "/kaw_net_move", send_move);
  printf ("Move service started.\n");
  ros::ServiceServer ser_getpos = n.advertiseService(ServerName + "/kaw_net_getpos", send_getpos);
  printf ("Get_Position service started.\n");
  ros::ServiceServer ser_speed = n.advertiseService(ServerName + "/kaw_net_speed", send_speed);
  printf ("Speed_Set service started.\n");
  ros::ServiceServer ser_accur = n.advertiseService(ServerName + "/kaw_net_accur", send_accur);
  printf ("Accuracy_Set service started.\n");
  ros::ServiceServer ser_accdec = n.advertiseService(ServerName + "/kaw_net_acdec", send_acdec);
  printf ("Accel_Decel_Set service started.\n");
  ros::ServiceServer ser_exec = n.advertiseService(ServerName + "/kaw_net_exec", send_exec);
  printf ("Exec service started\n");
  
  printf ("=======================================================================\n");
  printf ("Services awaiting commands...\n");
  ros::spin();
  
  return 0;
}
