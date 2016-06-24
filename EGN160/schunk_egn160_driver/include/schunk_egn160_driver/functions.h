#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <string>
#include <bitset>
#include "schunk_egn160_driver/serial.h"

using namespace std;

class Functions
{
public:
    Functions(Serial *);
    ~Functions();
    bool Reference();
    bool GetState(float);
    bool MovePos(float); 
    bool MoveVel(float);
    bool SetTargetVel(float);
    bool Stop();
    bool FastStop();
    bool Ack();
    
private:
    Serial *objSerial;
    strMsg msg;
    void FloatToHexad(unsigned char*, float);
};
