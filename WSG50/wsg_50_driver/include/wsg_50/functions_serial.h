//======================================================================
/**
 *  @file
 *  functions_serial.h
 *
 *
 */
//======================================================================


#ifndef FUNCTIONS_SERIAL_H_
#define FUNCTIONS_SERIAL_H_

#include <string>
#include <vector>

//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int homing_serial(void);
int move_serial(float, float, bool);
int stop_serial();
int ack_fault_serial(void);
int grasp_serial(float, float);
int release_serial(float, float);
int setAcceleration_serial(float);
int setGraspingForceLimit_serial(float);
int getFingerData_serial(int);
int setSoftLimits_serial(float, float);
int clearSoftLimits_serial();
int getSoftLimits_serial();

#endif
