#include <iostream>
#include "MARM.h"
#include <time.h>
using namespace std; 

/***************
a test of two module groups on one bus 
ARM_G1, ARM_G2
***************/

/*
ARM_G1::ARM_G1(){
	// define elements in module groups 
	this->moduleG1 = &module[4];              // moduleG1[0] = ID_10; moduleG1[1] = ID_11
	this->moduleNum = 2; 
	this->idStart = 12; 
	this->idEnd = 13; 
	InitializeCriticalSection(&cSec);
}; 

ARM_G1::~ARM_G1(){
	DeleteCriticalSection(&cSec);
};


void ARM_G1::checkStatus(int index, SMP smp){
	EnterCriticalSection(&cSec);
	if (moduleG1[index].checkError() == true){			
		smp.CMD_ACK(index); 
	}
	if (moduleG1[index].checkReferenced() == false){
		smp.CMD_REFERENCE(index); 
		// sleep(5000); 
	}
	LeaveCriticalSection(&cSec);
}
int ARM_G1::checkPos(int index, float pos){
	if (pos > MaxPos[index] || pos < MinPos[index])
	{
		cout<<"Module "<<index<<": Value of position is out of range: "<<MinPos[index]<<" ~ "<<MaxPos[index]<<endl;
		return OUT_RANGE;
	}
	else 
		return IN_RANGE; 
}
int ARM_G1::checkRelPos(int index, float relpos){
	float module_pos; 
	module_pos = moduleG1[index].getPos(); 
	if (relpos > ( MaxPos[index] - module_pos ) || relpos < ( MinPos[index] - module_pos) )
	{
		cout<<"Module "<<index<<": Value of relative position is out of range: "<<( MinPos[index] - module_pos)<<" ~ "<<( MaxPos[index] - module_pos )<<endl;
		return OUT_RANGE;
	}
	else 
		return IN_RANGE; 
}
float ARM_G1::checkVel(int index, float vel){
	if (vel >MaxVel[index]||vel< -MaxVel[index])
	{	
		cout<<"Module "<< index << ": velocity is out of range"<<endl; 
		cout<<"Set to default value"<<endl; 
		if (vel>0)
			vel = CustomedVel[index]; 
		else 
			vel = -CustomedVel[index];
	}
	return vel;
}; 
float ARM_G1::checkAcc(int index, float acc){
	if (acc >MaxAcc[index]||acc< -MaxAcc[index])
	{	
		cout<<"Module "<< index << ": acceleration is out of range"<<endl; 
		cout<<"Set to default value"<<endl; 
		if (acc>0)
			acc = CustomedAcc[index]; 
		else 
			acc = -CustomedAcc[index];
	}
	return acc;
}; 

float ARM_G1::checkCur(int index, float cur){
	if (cur >MaxAcc[index]||cur< -MaxAcc[index])
	{	
		cout<<"Module "<< index << ": current is out of range"<<endl; 
		cout<<"Set to default value"<<endl; 
		if (cur>0)
			cur = CustomedCurLimit[index]; 
		else 
			cur = -CustomedCurLimit[index];
	}
	return cur;
};



NTCAN_RESULT ARM_G1::CALL_CMD_MOV_POS(int index, SMP smp, float pos, float vel, float acc, float cur){
	
	NTCAN_RESULT retValue; 
	this->checkStatus(index, smp);
	this->checkPos(index, pos);
	this->checkVel(index, vel);
	this->checkAcc(index, acc); 
	this->checkCur(index, cur); 
	retValue = smp.CMD_MOV_POS(index, pos, vel, acc, cur); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Motion is paused! "<<endl; 
	}
	else 
	{	
		cout<<"Motion is accomplished! "<<endl; 
	}
	return retValue; 
};

NTCAN_RESULT ARM_G1::CALL_CMD_STATE_UPDATE(int index, SMP smp, float timeFreq, uint8_t mode){
	// call CMD_GET_STATE
	NTCAN_RESULT retValue; 
	retValue = smp.CMD_GET_STATE(index, timeFreq, mode); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Motion is paused! "<<endl; 
	}; 
	return retValue; 
};
NTCAN_RESULT ARM_G1::CALL_CMD_ACK(int index, SMP smp){
	NTCAN_RESULT retValue; 
	retValue = smp.CMD_ACK(index); 
	return retValue; 
};




ARM_G2::ARM_G2(){
	this->moduleG2 = &module[6]; 
	this->moduleNum = 1; 
	this->idStart = 14; 
	this->idEnd = 14; 
	InitializeCriticalSection(&cSec);
}; 


ARM_G2::~ARM_G2(){
	DeleteCriticalSection(&cSec);
}; 


void ARM_G2::checkStatus(int index, SMP smp){
	EnterCriticalSection(&cSec);
	if (moduleG2[index].checkError() == true){			
		smp.CMD_ACK(index); 
	}
	if (moduleG2[index].checkReferenced() == false){
		smp.CMD_REFERENCE(index); 
		// sleep(5000); 
	}
	LeaveCriticalSection(&cSec);
}
int ARM_G2::checkPos(int index, float pos){
	if (pos > MaxPos[index] || pos < MinPos[index])
	{
		cout<<"Module "<<index<<": Value of position is out of range: "<<MinPos[index]<<" ~ "<<MaxPos[index]<<endl;
		return OUT_RANGE;
	}
	else 
		return IN_RANGE; 
}
int ARM_G2::checkRelPos(int index, float relpos){
	float module_pos; 
	module_pos = moduleG2[index].getPos(); 
	if (relpos > ( MaxPos[index] - module_pos ) || relpos < ( MinPos[index] - module_pos) )
	{
		cout<<"Module "<<index<<": Value of relative position is out of range: "<<( MinPos[index] - module_pos)<<" ~ "<<( MaxPos[index] - module_pos )<<endl;
		return OUT_RANGE;
	}
	else 
		return IN_RANGE; 
}
float ARM_G2::checkVel(int index, float vel){
	if (vel >MaxVel[index]||vel< -MaxVel[index])
	{	
		cout<<"Module "<< index << ": velocity is out of range"<<endl; 
		cout<<"Set to default value"<<endl; 
		if (vel>0)
			vel = CustomedVel[index]; 
		else 
			vel = -CustomedVel[index];
	}
	return vel;
}; 
float ARM_G2::checkAcc(int index, float acc){
	if (acc >MaxAcc[index]||acc< -MaxAcc[index])
	{	
		cout<<"Module "<< index << ": acceleration is out of range"<<endl; 
		cout<<"Set to default value"<<endl; 
		if (acc>0)
			acc = CustomedAcc[index]; 
		else 
			acc = -CustomedAcc[index];
	}
	return acc;
}; 

float ARM_G2::checkCur(int index, float cur){
	if (cur >MaxAcc[index]||cur< -MaxAcc[index])
	{	
		cout<<"Module "<< index << ": current is out of range"<<endl; 
		cout<<"Set to default value"<<endl; 
		if (cur>0)
			cur = CustomedCurLimit[index]; 
		else 
			cur = -CustomedCurLimit[index];
	}
	return cur;
};

NTCAN_RESULT ARM_G2::CALL_CMD_MOV_POS(int index, SMP smp, float pos, float vel, float acc, float cur){
	
	NTCAN_RESULT retValue; 
	this->checkStatus(index, smp);
	this->checkPos(index, pos);
	this->checkVel(index, vel);
	this->checkAcc(index, acc); 
	this->checkCur(index, cur); 
	retValue = smp.CMD_MOV_POS(index, pos, vel, acc, cur); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Motion is paused! "<<endl; 
	}
	else 
	{	
		cout<<"Motion is accomplished! "<<endl; 
	}
	return retValue; 
};

NTCAN_RESULT ARM_G2::CALL_CMD_STATE_UPDATE(int index, SMP smp, float timeFreq, uint8_t mode){
	// call CMD_GET_STATE
	// mode: 0x01 - pos; 0x02 - vel; 0x07- cur; 
	NTCAN_RESULT retValue; 
	retValue = smp.CMD_GET_STATE(index, timeFreq, mode); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Motion is paused! "<<endl; 
	}; 
	return retValue; 
};
NTCAN_RESULT ARM_G2::CALL_CMD_ACK(int index, SMP smp){
	NTCAN_RESULT retValue; 
	retValue = smp.CMD_ACK(index); 
	return retValue; 
};

*/



ARM_G1::ARM_G1(){
	// define elements in module groups 
	this->moduleArm = &module[5];              // moduleArm[0] = ID_12; moduleArm[1] = ID_13
	this->moduleNum = 1; 
	this->idStart = 13; 
	this->idEnd = 13; 
	InitializeCriticalSection(&cSec);
};
ARM_G1::~ARM_G1(){
	DeleteCriticalSection(&cSec);
};

/*
ARM_G2::ARM_G2(){
	// define elements in module groups
	this->moduleArm = &module[6];              // moduleArm[0] = ID_14; 
	this->moduleNum = 1; 
	this->idStart = 14; 
	this->idEnd = 14; 
	InitializeCriticalSection(&cSec);
};
ARM_G2::~ARM_G2(){
	DeleteCriticalSection(&cSec);
};

NTCAN_RESULT ARM_G2::SINGLE_MODULE2STEP_MOTION(SMP smp, int index, float pos_01, float pos_02){
	// function for test 
	NTCAN_RESULT retValue; 
	int32_t module_id; 
	float module_pos, set_vel, set_acc; 
	float set_curLimit;     // it's a limit 
	clock_t timeStart, timeFinish, timeWait; 

	index = index + this->idStart - ID_START; 

	module_pos = module[index].getPos(); 
	this->checkPos(index, pos_01); 
	set_curLimit = CustomedCurLimit[index]; 
	set_vel = CustomedVel[index]; 
	set_acc = CustomedAcc[index]; 

	retValue = smp.CMD_MOV_POS(index, pos_01, set_vel, set_acc, set_curLimit); 
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(0); 
	}; 

	timeWait = 5; 
	timeStart = clock(); 
	while(1)
	{
		timeFinish = clock(); 
		if((timeFinish-timeStart)/(clock_t)1000>=8)
			break; 
	}

	module_pos = this->moduleArm[index].getPos(); 
	module_id = this->moduleArm[index].getId(); 
	// cout<<"Position 1 of module "<< module_id<<" is "<< module_pos <<" . "<<endl;

	this->checkPos(index, pos_02); 
	retValue = smp.CMD_MOV_POS(index, pos_02, set_vel, set_acc, set_curLimit);
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(1); 
	}; 
	
	timeWait = 5; 
	timeStart = clock(); 
	while(1)
	{
		timeFinish = clock(); 
		if((timeFinish-timeStart)/(clock_t)1000>=timeWait)
			break; 
	}; 
	
	
	module_pos = this->moduleArm[index].getPos(); 
	module_id = this->moduleArm[index].getId(); 
	// cout<<"Position 2 of module "<< module_id<<" is "<< module_pos <<" . "<<endl;

	retValue = smp.CMD_STOP(index); 
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(1); 
	}; 
	return retValue; 
}

*/


NTCAN_RESULT ARM_G1::DOUBLE_MODULE2STEP_MOTION(SMP smp, int index_01, int index_02, float pos_01, float pos_02){
	NTCAN_RESULT retValue; 
	clock_t timeStart, timeFinish, timeWait;
	float module_pos, set_vel, set_acc, set_curLimit;
	int32_t module_id; 

	index_01 = index_01 + this->idStart - ID_START;
	index_02 = index_02 + this->idStart - ID_START; 

	this->checkPos(index_01, pos_01); 
	set_curLimit = CustomedCurLimit[index_01]; 
	set_vel = CustomedVel[index_01]; 
	set_acc = CustomedAcc[index_01]; 	
	retValue = smp.CMD_MOV_POS(index_01, pos_01, set_vel, set_acc, set_curLimit);
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(0); 
	};

	timeWait = 5; 
	timeStart = clock(); 
	while(1)
	{
		timeFinish = clock(); 
		if((timeFinish-timeStart)/(clock_t)1000>=timeWait)
			break; 
	}; 

	module_pos = this->moduleArm[index_01].getPos(); 
	module_id = this->moduleArm[index_01].getId(); 
	// cout<<"Position 1 of module "<< module_id<<" is "<< module_pos <<" . "<<endl;

	this->checkPos(index_02, pos_02); 
	set_curLimit = CustomedCurLimit[index_02]; 
	set_vel = CustomedVel[index_02]; 
	set_acc = CustomedAcc[index_02]; 	
	retValue = smp.CMD_MOV_POS(index_02, pos_02, set_vel, set_acc, set_curLimit);
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(0); 
	};

	timeWait = 5; 
	timeStart = clock(); 
	while(1)
	{
		timeFinish = clock(); 
		if((timeFinish-timeStart)/(clock_t)1000>=timeWait)
			break; 
	}; 

	module_pos = this->moduleArm[index_02].getPos(); 
	module_id = this->moduleArm[index_02].getId(); 
	// cout<<"Position 2 of module "<< module_id<<" is "<< module_pos <<" . "<<endl;

	retValue = smp.CMD_STOP(index_01); 
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(0); 
	}; 
	retValue = smp.CMD_STOP(index_02); 
	if (retValue != NTCAN_SUCCESS){
		cout<<"Failed in the first step!"<<endl; 
		exit(0); 
	}; 
	return retValue; 

}