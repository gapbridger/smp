#include <iostream>
#include <time.h>
#include <windows.h>
// #include "ARM.h"
#include "MARM.h"
// #include<math.h>
using namespace std; 

/**********
communication test 

*********/

int main(){
	
	SMP smp; 

	int idx; 
	NTCAN_RESULT retValue = 0; 
	float msgFreq; 
	clock_t timeStart, timeFinish; 
	uint8_t msgMode; 
	int32_t moduleArm_id; 
	float moduleArm_pos; 
	
	int32_t moduleID[8] = {8, 9, 10, 11, 12, 13, 14, 15};   
	float valuePos[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueRelPos[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueVel[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueAcc[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 
	float valueCur[8] = {0, 0 ,0, 0, 0, 0 ,0, 0}; 

	

	// open CAN; set up threads

	smp.canON(); 
	smp.threadOpen(); 


	/*
	test 01 single communication
	1. polling (CMD_GET_STATE)
	2. main (send move cmd)
	*/

	/*
	ARM arm; 
	idx = 6;  // MODULE ID   //14
	
	valueRelPos[idx] = 10; 
	valueVel[idx] = 5; 
	valueAcc[idx] = 5; 
	valueCur[idx] = 4;  // refer to cur. on MCdemo 


	retValue = arm.CALL_CMD_ACK(idx, smp); 
	if (retValue != NTCAN_SUCCESS)
		 exit(1); 


	// CMD_STATE    10/s
	msgFreq = (float)0.1; 
	msgMode = 0x07; 
	arm.CALL_CMD_STATE_UPDATE(idx, smp, msgFreq, msgMode); 
	
	//move 1  0->10
	moduleArm_id = arm.moduleArm[idx].getId(); 
	moduleArm_pos = arm.moduleArm[idx].getPos(); 
	cout<<"Position 0(0) of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;
	cout<<"Perform CMD 01 ..." << endl; 
	valuePos[idx] = 10; 
	arm.CALL_CMD_MOV_POS(idx, smp, valuePos[idx], valueVel[idx], valueAcc[idx], valueCur[idx]); 
	timeStart = clock(); 
	while(1)
	{	
		timeFinish = clock(); 
		if ((timeFinish-timeStart)/(clock_t)1000>=5)
			break; 
	}	
	moduleArm_id = arm.moduleArm[idx].getId(); 
	moduleArm_pos = arm.moduleArm[idx].getPos(); 
	cout<<"Position 1(10) of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;
	moduleArm_pos = arm.moduleArm[idx].getPos(); 
	cout<<"Position 1(10) of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;

	//move 2: 10->-10;
	
	valuePos[idx] = -10; 
	arm.CALL_CMD_MOV_POS(idx, smp, valuePos[idx], valueVel[idx], valueAcc[idx], valueCur[idx]); 
	timeStart = clock(); 
	while(1)
	{	
		timeFinish = clock(); 
		if ((timeFinish-timeStart)/(clock_t)1000>=8)
			break; 
	}	
	moduleArm_pos = arm.moduleArm[idx].getPos(); 
	cout<<"Position 2(-10) of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;

	// mov 3: -10->0; 
	valuePos[idx] = 0; 
	arm.CALL_CMD_MOV_POS(idx, smp, valuePos[idx], valueVel[idx], valueAcc[idx], valueCur[idx]); 
	timeStart = clock(); 
	while(1)
	{	
		timeFinish = clock(); 
		if ((timeFinish-timeStart)/(clock_t)1000>=5)
			break; 
	}
	moduleArm_pos = arm.moduleArm[idx].getPos(); 
	cout<<"Position 3(0) of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;

	*/
	
	/*
	test 02 two module-groups communication 
	1. sequential motion by two modules
	*/
	ARM_G1 armG1;    // id_13, 14 (5,6);  
	// ARM_G2 armG2;    // id_14 (6); 
	
	// CMD_ACK
	idx = 0;     // id_13 
	retValue = armG1.CALL_CMD_ACK(idx, smp);
	idx = 1;     // id_14
	retValue = armG1.CALL_CMD_ACK(idx, smp); 
	// idx = 0; 
	// retValue = armG2.CALL_CMD_ACK(idx, smp); 


	// CMD_STATE    10/s
	msgFreq = (float)0.1; 
	msgMode = 0x07;
	idx = 0; 
	armG1.CALL_CMD_STATE_UPDATE(idx, smp, msgFreq, msgMode);
	idx = 1; 
	armG1.CALL_CMD_STATE_UPDATE(idx, smp, msgFreq, msgMode);
	
	
	// motion 01  id_14(armG2) 2 step
	//idx = 0; 
	//valuePos[6] = 10; 
	//moduleArm_id = armG2.moduleArm[idx].getId(); 
	//moduleArm_pos = armG2.moduleArm[idx].getPos(); 
	//cout<<"Position 0 of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;
	//cout<<"Perform CMD 01 ..." << endl; 
	//retValue = armG2.SINGLE_MODULE2STEP_MOTION(smp, idx, valuePos[6], -valuePos[6]); 

	//moduleArm_id = armG2.moduleArm[idx].getId(); 
	//moduleArm_pos = armG2.moduleArm[idx].getPos(); 
	//cout<<"Position 1 of module "<< moduleArm_id<<" is "<< moduleArm_pos <<endl;
	
    
	// motion 02  id_12(armG1), id_13(armG1) 2 step
	moduleArm_id = armG1.moduleArm[0].getId(); 
	moduleArm_pos = armG1.moduleArm[0].getPos(); 
	cout<<"Position 0 of module "<< moduleArm_id << " is "<< moduleArm_pos<<endl; 
	moduleArm_id = armG1.moduleArm[1].getId(); 
	moduleArm_pos = armG1.moduleArm[1].getPos(); 
	cout<<"Position 0 of module "<< moduleArm_id << " is "<< moduleArm_pos<<endl; 

	cout<<"Perform CMD 02 ..."<<endl; 
	valuePos[5] = 0;
	valuePos[6] = 0; 
	retValue = armG1.DOUBLE_MODULE2STEP_MOTION(smp, 0, 1, valuePos[5], valuePos[6]); 

	moduleArm_id = armG1.moduleArm[0].getId(); 
	moduleArm_pos = armG1.moduleArm[0].getPos(); 
	cout<<"Position 1 of module "<< moduleArm_id << " is "<< moduleArm_pos<<endl; 
	moduleArm_id = armG1.moduleArm[1].getId(); 
	moduleArm_pos = armG1.moduleArm[1].getPos(); 
	cout<<"Position 1 of module "<< moduleArm_id << " is "<< moduleArm_pos<<endl; 


	


	
	
	
	smp.threadClose(); 
	smp.canOFF(); 
	return 1; 
};

