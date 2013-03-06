#include <iostream>
#include "ARM.h"
using namespace std; 

/*************
class ARM
1. range & limit (pos, relPos, vel, acc, cur)
2. motion cmd 
**************/

ARM::ARM(){
	// handle = NULL;
	// baud = NTCAN_BAUD_1000; 
	this->moduleArm = &module[0]; 
	this->moduleNum = 8; 
	this->idStart = 8; 
	this->idEnd = 15; 
	InitializeCriticalSection(&cSec);
}

ARM::~ARM(){
	 DeleteCriticalSection(&cSec);
	 // delete this->moduleArm; 
	// this->threadClose(); 
}


void ARM::checkStatus(int index, SMP smp){
	EnterCriticalSection(&cSec);
	if (moduleArm[index].checkError() == true){			
		smp.CMD_ACK(index); 
	}
	if (moduleArm[index].checkReferenced() == false){
		smp.CMD_REFERENCE(index); 
		// sleep(5000); 
	}
	LeaveCriticalSection(&cSec);
}
int ARM::checkPos(int index, float pos){
	if (pos > MaxPos[index] || pos < MinPos[index])
	{
		cout<<"Module "<<index<<": Value of position is out of range: "<<MinPos[index]<<" ~ "<<MaxPos[index]<<endl;
		return OUT_RANGE;
	}
	else 
		return IN_RANGE; 
}
int ARM::checkRelPos(int index, float relpos){
	float module_pos; 
	module_pos = moduleArm[index].getPos(); 
	if (relpos > ( MaxPos[index] - module_pos ) || relpos < ( MinPos[index] - module_pos) )
	{
		cout<<"Module "<<index<<": Value of relative position is out of range: "<<( MinPos[index] - module_pos)<<" ~ "<<( MaxPos[index] - module_pos )<<endl;
		return OUT_RANGE;
	}
	else 
		return IN_RANGE; 
}
float ARM::checkVel(int index, float vel){
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
float ARM::checkAcc(int index, float acc){
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

float ARM::checkCur(int index, float cur){
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

/*
NTCAN_RESULT ARM::moduleIdAdd(NTCAN_HANDLE handle){
	NTCAN_RESULT	retValue; 
	int32_t id = 0; 
	int module_size = this->moduleArraySize; 
	
	for (int i = 0; i<module_size; i++){
		id = module[i].getId(); 
		id = id|MSG_MASTER_TO_SLAVE;
		retValue = canIdAdd(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdAdd() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdAdd() added Module ["<< i << "]|M->S to CAN. "<<endl;  
		}
		
		id = id|MSG_SLAVE_TO_MASTER;
		retValue = canIdAdd(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdAdd() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdAdd() added Module ["<< i << "]|S->M to CAN. "<<endl;  
		}

		id = id|MSG_ERROR;
		retValue = canIdAdd(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdAdd() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdAdd() added Module ["<< i << "]|Err(S->M) to CAN. "<<endl;  
		}
	}
	return NTCAN_SUCCESS; 
}

NTCAN_RESULT ARM::moduleIdDelete(){
	NTCAN_RESULT	retValue; 
	int32_t id = 0; 
	int module_size = this->moduleArraySize ; 
	for (int i = 0; i<module_size; i++){
		id = module[i].getId(); 
		id = id|MSG_MASTER_TO_SLAVE;
		retValue = canIdDelete(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdDelete() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdDelete() deleted Module ["<< i << "]|M->S in CAN. "<<endl;  
		}

		id = id|MSG_SLAVE_TO_MASTER;
		retValue = canIdDelete(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdDelete() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdDelete() deleted Module ["<< i << "]|S->M in CAN. "<<endl;  
		}

		id = id|MSG_ERROR;
		retValue = canIdDelete(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdDelete() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdAdd() deleted Module ["<< i << "]|Err(S->M) in CAN. "<<endl;  
		}
	}
	return NTCAN_SUCCESS; 
}

*/

NTCAN_RESULT ARM::CALL_CMD_MOV_POS(int index, SMP smp, float pos, float vel, float acc, float cur){
	
	NTCAN_RESULT retValue; 
	index = index + this->idStart - ID_START; 
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

NTCAN_RESULT ARM::CALL_CMD_STATE_UPDATE(int index, SMP smp, float timeFreq, uint8_t mode){
	// call CMD_GET_STATE
	// mode: 0x01 - pos; 0x02 - vel; 0x07- cur; 
	NTCAN_RESULT retValue; 
	index = index + this->idStart - ID_START; 
	retValue = smp.CMD_GET_STATE(index, timeFreq, mode); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Motion is paused! "<<endl; 
	}; 
	return retValue; 
};
NTCAN_RESULT ARM::CALL_CMD_ACK(int index, SMP smp){
	NTCAN_RESULT retValue; 
	index = index + this->idStart - ID_START; 
	retValue = smp.CMD_ACK(index); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Failed in CALL_CMD_ACK. "<<endl; 
		exit(1); 
	}
	return retValue; 
};
NTCAN_RESULT ARM::CALL_CMD_STOP(int index, SMP smp){
	NTCAN_RESULT retValue; 
	index = index +this->idStart - ID_START; 
	retValue = smp.CMD_STOP(index); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"Failed in CALL_CMD_STOP. "<<endl; 
		exit(1); 
	}
	return retValue; 
}; 

