#include "schunk_egn160_driver/functions.h"

Functions::Functions(Serial *objSerial_)
{
    this->objSerial = objSerial_;
    this->msg.state = 0x05;
    this->msg.gripper = 0x0C;
}

Functions::~Functions()
{
}

bool Functions::Reference()
{
    this->msg.len = 0x01;
    this->msg.cmd = 0x92; 
    if(objSerial->Write(msg) == 6)  return true;
    else                            return false;    
}

bool Functions::GetState(float period)  // [s]
{
    this->msg.len = 0x06;
    this->msg.cmd = 0x95;
    FloatToHexad(this->msg.param, period);    
    this->msg.param[4] = 0x07;
    if(objSerial->Write(msg) == 11)  return true;
    else                             return false;
}

bool Functions::MovePos(float l)    // [mm]
{
    this->msg.len = 0x05;
    this->msg.cmd = 0xB0;
    FloatToHexad(this->msg.param, l);
    if(objSerial->Write(msg) == 10)  return true;
    else                             return false;   
}

bool Functions::MoveVel(float v)    // [mm/s]
{
    this->msg.len = 0x05;
    this->msg.cmd = 0xB5;
    FloatToHexad(this->msg.param, v);
    if(objSerial->Write(msg) == 10)  return true;
    else                             return false;  
}

bool Functions::SetTargetVel(float v)    // [mm/s]
{
    this->msg.len = 0x05;
    this->msg.cmd = 0xA0;
    FloatToHexad(this->msg.param, v);
    if(objSerial->Write(msg) == 10)  return true;
    else                             return false;    
}

bool Functions::Stop()   
{
    this->msg.len = 0x01;
    this->msg.cmd = 0x91;
    if(objSerial->Write(msg) == 6)  return true;
    else                            return false;    
}

bool Functions::FastStop()   
{
    this->msg.len = 0x01;
    this->msg.cmd = 0x90;
    if(objSerial->Write(msg) == 6)  return true;
    else                            return false;    
}

bool Functions::Ack()   
{
    this->msg.len = 0x01;
    this->msg.cmd = 0x8B;
    if(objSerial->Write(msg) == 6)  return true;
    else                            return false;    
}

void Functions::FloatToHexad(unsigned char* param, float data)
{
    union 
    {
        float num;
        unsigned char chars[4];
    } val;
    
    val.num = data;
    
    for(int i = 0; i < 4; i++)
        param[i] = val.chars[i];
}
