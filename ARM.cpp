#include <iostream>
#include "Darm.h"

ARM::ARM(){
	handle = NULL;
	baud = NTCAN_BAUD_1000; 
	int idx=0; 
	// initialization 
	for (int i=ID_START; i<=ID_END; i++)
	{
		module[idx].id = i;
		module[idx].status = 0x00;   // ref, moving, prog.mode, warning, err, brake, move.end, pos.reached 
		module[idx].errorCode = 0x00;
		module[idx].getStateMode = 0x00; 

		module[idx].position = 0;		//original
		module[idx].velocity = 0;
		module[idx].current = 0;
		
		module[idx].referenced = false; 
		module[idx].moving = false; 
		module[idx].error = false;	
		module[idx].brake = true; 
		module[idx].moveEnd = true; 
		module[idx].posReached = true; 

		// module[idx].hold = false; 
		module[idx].movBack = false; 
		module[idx].movForth = false; 
		idx++; 
	}

	for(int i = 0; i<8; i++)
		fragLen[i]=0; 

	pos_buffer=(uint8_t*)new uint8_t[4];
	vel_buffer=(uint8_t*)new uint8_t[4];
	cur_buffer=(uint8_t*)new uint8_t[4];
	
	InitializeCriticalSection(&cSec);
	canCommActive = false;
	canPollActive = true;

	this->threadOpen(); 
}

ARM::~ARM(){
	DeleteCriticalSection(&cSec);	
	this->threadClose(); 
}


void ARM::checkStatus(int index){
	EnterCriticalSection(&cSec);
	if (module[index].error = true)
		this->CMD_ACK(index); 
	if (module[index].referenced = false){
		this->CMD_REFERENCE(index); 
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
	if (relpos > ( MaxPos[index] - module[index].position ) || relpos < ( MinPos[index] - module[index].position ) )
	{
		cout<<"Module "<<index<<": Value of relative position is out of range: "<<( MinPos[index] - module[index].position)<<" ~ "<<( MaxPos[index] - module[index].position )<<endl;
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
			cur = CustomedCur[index]; 
		else 
			cur = -CustomedCur[index];
	}
	return cur;
};

