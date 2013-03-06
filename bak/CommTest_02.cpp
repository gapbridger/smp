#include"MARM.h"
#include<iostream>
#include<time.h>
#include<windows.h>
// #include<math.h>
using namespace std; 

void main(){
	SMP smp; 
	ARM arm; 

	int idx; 
	NTCAN_RESULT retValue; 
	float msgFreq = (float)0.1; 
	clock_t timeStart, timeFinish; 
	uint8_t msgMode = 0x07; 
	int32_t moduleArm_id; 
	float moduleArm_pos; 

	float valuePos[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueRelPos[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueVel[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueAcc[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueCur[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 

	idx = 6;  // MODULE ID   //14
	valuePos[idx] = 10; 
	valueRelPos[idx] = 10; 
	valueVel[idx] = 5; 
	valueAcc[idx] = 5; 
	valueCur[idx] = 4;  // refer to cur. on MCdemo 

	// open CAN; set up threads
	smp.canON(); 
	smp.threadOpen(); 

	retValue = arm.CALL_CMD_ACK(idx, smp); 
	if (retValue != NTCAN_SUCCESS)
		 exit(1); 

	arm.CALL_CMD_STATE_UPDATE(idx, smp, msgFreq, msgMode); 
	
	//move 1 
	moduleArm_id = arm.moduleArm[idx].getId(); 
	moduleArm_pos = arm.moduleArm[idx].getPos(); 
	cout<<"Position 0 of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;
	cout<<"Perform CMD 01 ..." << endl; 
	arm.CALL_CMD_MOV_POS(idx, smp, valuePos[idx], valueVel[idx], valueAcc[idx], valueCur[idx]); 
	timeStart = clock(); 
	while(1)
	{	
		timeFinish = clock(); 
		if ((timeFinish-timeStart)/(clock_t)1000>=5)
		{
			moduleArm_id = arm.moduleArm[idx].getId(); 
			moduleArm_pos = arm.moduleArm[idx].getPos(); 
			cout<<"Position 1 of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;
			break; 
		}
	}	
}

