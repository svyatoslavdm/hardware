//======================================================================
/**
 *  @file
 *  functions_serial.cpp
 *  
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the and Weiss Robotics GmbH nor the names of its 
 *       contributors may be used to endorse or promote products derived from
 *	 this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
//======================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <string>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"
#include "wsg_50/functions_serial.h"

int homing_serial( void )
{
	status_t status;
	int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;

	payload[0] = 0x00;

    // Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x20; msg.len = 1; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command HOMING\n");
        return -1;
    }

	return 0;
}


/** \brief  Send move command (0x21) to gripper
 *  \param  ignore_response Do not read back response from gripper. (Must be read elsewhere, for auto update.)
 */
int move_serial(float width, float speed, bool stop_on_block)
{
	status_t status;
	int res;
	unsigned char payload[9];
	unsigned char *resp;
	unsigned int resp_len;

	// Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
	payload[0] = 0x00;
	if (stop_on_block) payload[0] |= 0x02;

	// Copy target width and speed
	memcpy(&payload[1], &width, sizeof( float ) );
	memcpy(&payload[5], &speed, sizeof( float ) );

    // Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x21; msg.len = 9; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command MOVE\n");
        return -1;
    }
    
	return 0;
}


int stop_serial()
{
	status_t status;
	int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;

	//payload[0] = 0x00;

    // Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x22; msg.len = 0; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command STOP\n");
        return -1;
    }
    
    return 0;
}


int ack_fault_serial( void )
{
	status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

	payload[0] = 0x61;  
	payload[1] = 0x63;
	payload[2] = 0x6B;

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x24; msg.len = 3; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command ACK\n");
        return -1;
    }
    
	return 0;
}

int grasp_serial( float objWidth, float speed )
{
	status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy part width and speed
	memcpy( &payload[0], &objWidth, sizeof( float ) );
	memcpy( &payload[4], &speed, sizeof( float ) );

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x25; msg.len = 8; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command GRASP\n");
        return -1;
    }
    
	return( 0 );
}


int release_serial( float width, float speed )
{
	status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy part width and speed
	memcpy( &payload[0], &width, sizeof( float ) );
	memcpy( &payload[4], &speed, sizeof( float ) );

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x26; msg.len = 8; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command RELEASE\n");
        return -1;
    }
    
	return 0;
}


int setAcceleration_serial( float acc )
{
	status_t status;
	int res;
	unsigned char payload[4];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy target width and speed
	memcpy( &payload[0], &acc, sizeof( float ) );

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x30; msg.len = 4; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command SET ACCELERATION\n");
        return -1;
    }

	return 0;
}

int setGraspingForceLimit_serial( float force )
{
	status_t status;
	int res;
	unsigned char payload[4];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy target width and speed
	memcpy( &payload[0], &force, sizeof( float ) );

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x32; msg.len = 4; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command SET GRASPING FORCE LIMIT\n");
        return -1;
    }

	return 0;
}

int getFingerData_serial(int index)
{
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char *resp;
    unsigned int resp_len;
    unsigned char vResult[4];

    if (index > 1)  return 0; 
    
	memcpy( &payload, &index, 1);

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x63; msg.len = 1; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command GET FINGER DATA\n");
        return -1;
    }
	
	return 0;
}

int setSoftLimits_serial(float limit_minus, float limit_plus)
{
	status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

	// Copy target width and speed
	memcpy( &payload[0], &limit_minus, sizeof( float ) );
	memcpy( &payload[4], &limit_plus, sizeof( float ) );

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x34; msg.len = 8; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command SET SOFT LIMITS\n");
        return -1;
    }

	return 0;
}

int getSoftLimits_serial()
{
    status_t status;
    int res;
    unsigned char payload[1];
    unsigned char *resp;
    unsigned int resp_len;
    unsigned char limit_minus[4];
    unsigned char limit_plus[4];
        
	memset(payload, 0, 1);

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x35; msg.len = 0; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command GET SOFT LIMITS\n");
        return -1;
    }
//	
//	
//	limit_minus[0] = resp[2];
//	limit_minus[1] = resp[3];
//	limit_minus[2] = resp[4];
//	limit_minus[3] = resp[5];
//	limit_plus[0] = resp[6];
//	limit_plus[1] = resp[7];
//	limit_plus[2] = resp[8];
//	limit_plus[3] = resp[9];
//	
//	free( resp );
//	
//	dbgPrint( "Soft limits: LIMIT MINUS  %f \n                             LIMIT PLUS   %f\n", convert(limit_minus), convert(limit_plus));
//	
	return 0;
}

int clearSoftLimits_serial()
{
	status_t status;
	int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;
	
	memset(payload, 0, 1);

	// Submit command, do not wait for response
    msg_t msg;
    msg.id = 0x36; msg.len = 0; msg.data = &payload[0];
    res = msg_send(&msg);
    if (res <= 0) 
    {
        dbgPrint("Failed to send command CLEAR SOFT LIMITS\n");
        return -1;
    }
	
	return 0;
}
