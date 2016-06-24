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
