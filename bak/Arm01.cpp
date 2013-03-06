#include <iostream>
#include <windows.h>
#include <time.h>
#include "Arm01.h"
using namespace std; 

ARM::ARM(){
	handle = NULL;
	baud = NTCAN_BAUD_1000; 
	int idx=0; 
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
	eventActive = true;
};

ARM::~ARM()
{
	DeleteCriticalSection(&cSec);	
};

NTCAN_HANDLE ARM::canConnect(uint32_t baud){
	NTCAN_RESULT retValue; 
	int net = 0; 
	uint32_t mode = 0; 
	int32_t txQueueSize = 40; 
	int32_t rxQueueSize = 40; 
	int32_t txTimeout = 100; 
	int32_t rxTimeout = 1000; 

	retValue = canOpen(net, mode, txQueueSize, rxQueueSize, txTimeout, rxTimeout, &handle);
	if (retValue != NTCAN_SUCCESS){
		cout<<"canOpen() failed with error "<<retValue<<" !"<<endl;
		return(NULL);
	}
	else 
		cout<<"Function canOpen() returned OK."<<endl;
	
	retValue = canSetBaudrate(handle, baud);

	if (retValue != NTCAN_SUCCESS){
		cout<<"canSetBaudrate() failed with error "<<retValue<<" !"<<endl;
		canClose(handle);
		return(NULL);
	}
	else 
		cout<<"Function canSetBaudrate() returned OK."<<endl;

	return handle;  //
}; 

NTCAN_RESULT ARM::canScanBus(NTCAN_HANDLE handle){
	NTCAN_RESULT retValue;
	int32_t id=0;

	for (int32_t i = ID_START; i <= ID_END; i++){
		id = i|MSG_MASTER;
		retValue = canIdAdd(handle, id);
		if (retValue != NTCAN_SUCCESS){
			cout << "canIdAdd() failed with error " << retValue << " !" << endl;
			return(NTCAN_ID_NOT_ENABLED);
		}
		else{
			cout << "Function canIdAdd() added Module [" << i << "]|M->S to CAN. " << endl;
		}

		id = i|MSG_SLAVE;
		retValue = canIdAdd(handle, id);
		if (retValue != NTCAN_SUCCESS){
			cout << "canIdAdd() failed with error " << retValue << " !" << endl;
			return(NTCAN_ID_NOT_ENABLED);
		}
		else{
			cout << "Function canIdAdd() added Module [" << i << "]|S->M to CAN. " << endl;
		}

		id = i|MSG_ERROR;
		retValue = canIdAdd(handle, id);
		if (retValue != NTCAN_SUCCESS){
			cout << "canIdAdd() failed with error " << retValue << " !" << endl;
			return(NTCAN_ID_NOT_ENABLED);
		}
		else{
			cout << "Function canIdAdd() added Module [" << i << "]|Err(S->M) to CAN. " << endl;
		}
	}

	return NTCAN_SUCCESS;
}

NTCAN_RESULT ARM::canInitial(){
	NTCAN_RESULT retValue;
	
	handle = canConnect(baud);
	if (handle == NULL){
		cout << "NTCAN net not found!" << endl;
		return(NTCAN_NET_NOT_FOUND);
	}

	retValue = canScanBus(handle);
	if (retValue != NTCAN_SUCCESS){
		cout << "Scan Bus failed, ID not found!" << endl;
		return(NTCAN_ID_NOT_ENABLED);
	}	
	else 
		cout <<"CAN bus connected. "<<endl; 
	return NTCAN_SUCCESS;  	
}
NTCAN_RESULT ARM::canDisconnect(){
	NTCAN_RESULT retValue;
	int32_t id=0;

	for (int32_t i = ID_START; i <= ID_END; i++){
		id = i|MSG_MASTER;
		retValue = canIdDelete(handle, id); // delete master id
		if (retValue != NTCAN_SUCCESS){
			cout << "canIdDelete() failed with error " << retValue << " !" << endl;
			return(NTCAN_WRONG_DEVICE_STATE);
		}
		else{
			cout << "Function canIDDelete() deleted Module [" << i << "]|M->S in CAN. " << endl;
		}

		id = i|MSG_SLAVE;
		retValue = canIdDelete(handle, id); // delete slave id
		if (retValue != NTCAN_SUCCESS){
			cout << "canIdDelete() failed with error " << retValue << " !" << endl;
			return(NTCAN_WRONG_DEVICE_STATE);
		}
		else{
			cout << "Function canIDDelete() deleted Module [" << i << "]|M->S in CAN. " << endl;
		}

		id = i|MSG_ERROR;
		retValue = canIdDelete(handle, id); // delete error id
		if (retValue != NTCAN_SUCCESS){
			cout << "canIdDelete() failed with error " << retValue << " !" << endl;
			return(NTCAN_WRONG_DEVICE_STATE);
		}
		else{
			cout << "Function canIDDelete() deleted Module [" << i << "]|M->S in CAN. " << endl;
		}
	}

	retValue = canClose(handle);               //close can
	if (retValue != NTCAN_SUCCESS){
		cout << "NTCAN net not closed!" << endl;
		return(NTCAN_WRONG_DEVICE_STATE);
	}
	else{
		cout << "CAN bus disconnected. " << endl;
	}
	return NTCAN_SUCCESS;
}

NTCAN_RESULT ARM::CMD_ACK(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	
	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;
	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=0x8B;

	// retValue = canWrite(handle, &cmsg, &len, NULL); // acknowledgement
	retValue = canSend(handle, &cmsg, &len); // acknowledgement
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_ACK(acknowledge pending error code)!" << retValue << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}

// CMD_GET_STATE: 0x95 
//NTCAN_RESULT ARM::CMD_GET_STATE(int index)
//NTCAN_RESULT ARM::CMD_GET_STATE(int index, float time)
NTCAN_RESULT ARM::CMD_GET_STATE(int index, float time, uint8_t mode){
	// parameter: none; time; time, mode
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_time = (uint8_t*)new uint8_t[4];
	
	for (int i = 0; i < 8; i++)
			cmsg.data[i] = 0;
	cmsg.id = module[index].id|MSG_MASTER;
	if(time == 0){
		cmsg.len = 2;		
		cmsg.data[0] = 0x01; 
		cmsg.data[1] = _GET_STATE; //0x95
	}
	else{
		cmsg.len=7;
		float2Bit(time, param_time);
		cmsg.data[0] = 0x06;   
		cmsg.data[1] = _GET_STATE; 
		for (int i=2;i<6;i++)
			cmsg.data[i] = param_time[i-2];		
		cmsg.data[6] = mode;

		module[index].getStateMode = mode;  
	} 

	//retValue = canWrite(handle, &cmsg, &len, NULL);
	retValue = canSend(handle, &cmsg, &len);
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_GET_STATE(get state of module)! Error " << retValue << endl;
		return NTCAN_TX_ERROR;
	}
	delete[] param_time;
	return retValue;
}

NTCAN_RESULT ARM::CMD_REFERENCE(int index){
	// Caution: need to know zero position of each module 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;	
	retValue = CMD_ACK(index);//CMD ACK first 

	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;
	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=_CMD_REFERENCE;

	// retValue = canWrite(handle, &cmsg, &len, NULL); // reference
	retValue = canSend(handle, &cmsg, &len); // reference

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_REFERENCE!" << endl;
		return NTCAN_TX_ERROR;
	}
	else 
	{
		cout << "Successfully referenced" << endl; 
	}
	Sleep(5000);  // 
	return retValue;
}

NTCAN_RESULT ARM::CMD_STOP(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;
	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=_CMD_STOP;
	
	// retValue = canWrite(handle, &cmsg, &len, NULL); 
	retValue = canSend(handle, &cmsg, &len); 

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_STOP!" << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}

NTCAN_RESULT ARM::CMD_EMERGENCY_STOP(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	
	cmsg.len=2;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x01;
	cmsg.data[1]=_CMD_EMERGENCY_STOP;
	
	// retValue = canWrite(handle, &cmsg, &len, NULL);
	retValue = canSend(handle, &cmsg, &len);

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_EMERGENCY_STOP!" << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
}


/*
In new version, "pos" becomes optional parameter; SET_TARGET_POS could be called 
before MOV cmd. 
*/
NTCAN_RESULT ARM::CMD_MOV_POS(int index, float pos){
	// parameter: index, pos (previous values)
	// check range
	this->checkStatus(index); 
	this->checkPos(index, pos); 
	// vel = module[index].velocity; 
	// vel = this->checkVel(index, vel); 
	// module[index].velocity = vel; 

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];

	float2Bit(pos, param_pos);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_POS;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_pos[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);
	EnterCriticalSection(&cSec);
	if(pos > module[index].position){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);

	retValue = canSend(handle, &cmsg, &len);

	delete[] param_pos;
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;
}
NTCAN_RESULT ARM::CMD_MOV_POS(int index, float pos ,float vel, float acc, float cur){
	// parameter: index, pos, {vel, acc, cur} 
	this->checkStatus(index); 
	this->checkPos(index, pos); 
	vel = this->checkVel(index, vel); 
	acc = this->checkAcc(index, acc); 
	cur = this->checkCur(index, cur);

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[3];
	int32_t len=3;
	uint8_t direction=0x05;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_acc=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];

	float2Bit(pos, param_pos);
	float2Bit(vel, param_vel);
	float2Bit(acc, param_acc);
	float2Bit(cur, param_cur);

	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
		cmsg[2].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;
	cmsg[2].id = module[index].id|MSG_MASTER;

	cmsg[0].len=8;
	cmsg[1].len=8;
	cmsg[2].len=7;

	cmsg[0].data[0]=0x11; // length of each message data
	cmsg[1].data[0]=0x0B;
	cmsg[2].data[0]=0x05;

	cmsg[0].data[1]=0x84; // fragmentation
	cmsg[1].data[1]=0x85;
	cmsg[2].data[1]=0x86;

	cmsg[0].data[2]=_MOV_POS; // command

	for (int i=3;i<7;i++)
		cmsg[0].data[i]=param_pos[i-3];
	
	cmsg[0].data[7]=param_vel[0];
	for (int i=2;i<5;i++)
		cmsg[1].data[i]=param_vel[i-1];

	for (int i=5;i<8;i++)
		cmsg[1].data[i]=param_acc[i-5];
	cmsg[2].data[2]=param_acc[3];
	
	for(int i=3;i<7;i++)
		cmsg[2].data[i]=param_cur[i-3];
		
	//retValue = canWrite(handle, cmsg, &len, NULL);
	EnterCriticalSection(&cSec);
	if(pos > module[index].position){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);
	
	retValue = canSend(handle, cmsg, &len);
	
	delete[] cmsg;
	delete[] param_pos;
	delete[] param_vel;
	delete[] param_acc;
	delete[] param_cur;
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS! Error "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;
}
NTCAN_RESULT ARM::CMD_MOV_POS_REL(int index, float pos){

	this->checkStatus(index); 
	this->checkRelPos(index, pos); 
	// vel = module[index].velocity;  
	// this->checkVel(index, vel);  

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	float2Bit(pos, param_pos);
	
	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_POS_REL;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_pos[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);
	EnterCriticalSection(&cSec);
	if(pos > 0){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);
	retValue = canSend(handle, &cmsg, &len);
	delete[] param_pos;
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS_REL, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;

}; 
NTCAN_RESULT ARM::CMD_MOV_POS_REL(int index, float pos, float vel, float acc, float cur){

	this->checkStatus(index); 
	this->checkRelPos(index, pos); 
	vel = this->checkVel(index, vel);
	acc = this->checkAcc(index,acc);
	cur = this->checkCur(index,cur);

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[3];
	int32_t len=3;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_acc=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];
	float2Bit(pos, param_pos);
	float2Bit(vel, param_vel);
	float2Bit(acc, param_acc);
	float2Bit(cur, param_cur);
	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
		cmsg[2].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;
	cmsg[2].id = module[index].id|MSG_MASTER;
	cmsg[0].len=8;
	cmsg[1].len=8;
	cmsg[2].len=7;

	cmsg[0].data[0]=0x11; // length of each message data
	cmsg[1].data[0]=0x0B;
	cmsg[2].data[0]=0x05;

	cmsg[0].data[1]=0x84; // fragmentation
	cmsg[1].data[1]=0x85;
	cmsg[2].data[1]=0x86;

	cmsg[0].data[2]=_MOV_POS_REL; // command

	for (int i=3;i<7;i++)
		cmsg[0].data[i]=param_pos[i-3];

	cmsg[0].data[7]=param_vel[0];
	for (int i=2;i<5;i++)
		cmsg[1].data[i]=param_vel[i-1];

	for (int i=5;i<8;i++)
		cmsg[1].data[i]=param_acc[i-5];
	cmsg[2].data[2]=param_acc[3];

	for(int i=3;i<7;i++)
		cmsg[2].data[i]=param_cur[i-3];
			
	retValue = canSend(handle, cmsg, &len);
	
	EnterCriticalSection(&cSec);
	if(pos > 0){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);

	delete[] cmsg;
	delete[] param_pos;
	delete[] param_vel;
	delete[] param_acc;
	delete[] param_cur;
	
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS_REL, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;
}; 


NTCAN_RESULT ARM::CMD_MOV_POS_TIME(int index, float pos){
	float time = 5; 
	this->checkStatus(index);
	this->checkPos(index, pos); 
	// time = this->checkTime(index, pos, time); 
	
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	float2Bit(pos, param_pos);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_POS_TIME;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_pos[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);
	
	retValue = canSend(handle, &cmsg, &len);
	EnterCriticalSection(&cSec);
	if(pos > module[index].position){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);
	delete[] param_pos;
	
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS_TIME, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;	
}
NTCAN_RESULT ARM::CMD_MOV_POS_TIME(int index, float pos, float vel, float acc, float cur, float time){
	
	this->checkStatus(index);
	this->checkPos(index, pos); 
	vel = this->checkVel(index,vel);
	acc = this->checkAcc(index,acc);
	cur = this->checkCur(index,cur);
	// time = this->checkTime(index, pos, time); 

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[4];
	int32_t len=4;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_acc=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];
	uint8_t *param_time=(uint8_t*)new uint8_t[4];
	float2Bit(pos, param_pos);
	float2Bit(vel, param_vel);
	float2Bit(acc, param_acc);
	float2Bit(cur, param_cur);
	float2Bit(time, param_time);

	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
		cmsg[2].data[i] = 0;
		cmsg[3].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;
	cmsg[2].id = module[index].id|MSG_MASTER;
	cmsg[3].id = module[index].id|MSG_MASTER;
	cmsg[0].len=8;
	cmsg[1].len=8;
	cmsg[2].len=8;
	cmsg[3].len=3;

	cmsg[0].data[0]=0x15; // length of each message data
	cmsg[1].data[0]=0x0F;
	cmsg[2].data[0]=0x09;
	cmsg[3].data[0]=0x03;

	cmsg[0].data[1]=0x84; // fragmentation
	cmsg[1].data[1]=0x85;
	cmsg[2].data[1]=0x85;
	cmsg[3].data[1]=0x86;

	cmsg[0].data[2]=_MOV_POS_TIME; // command

	for (int i=3;i<7;i++)
		cmsg[0].data[i]=param_pos[i-3];

	cmsg[0].data[7]=param_vel[0];
	for (int i=2;i<5;i++)
		cmsg[1].data[i]=param_vel[i-1];

	for (int i=5;i<8;i++)
		cmsg[1].data[i]=param_acc[i-5];
	cmsg[2].data[2]=param_acc[3];

	for(int i=3;i<7;i++)
		cmsg[2].data[i]=param_cur[i-3];

	cmsg[2].data[7]=param_time[0];
	for(int i=2;i<5;i++)
		cmsg[3].data[i]=param_time[i-1];
			
	retValue = canSend(handle, cmsg, &len);

	EnterCriticalSection(&cSec);
	if(pos > module[index].position){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);
	
	delete[] cmsg;
	delete[] param_pos;
	delete[] param_vel;
	delete[] param_acc;
	delete[] param_cur;
	delete[] param_time;
	

	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS_TIME, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}

	return retValue;
}
NTCAN_RESULT ARM::CMD_MOV_POS_TIME_REL(int index, float pos){
	this->checkStatus(index);
	this->checkRelPos(index, pos); 

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];

	float2Bit(pos, param_pos);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_POS_TIME_REL;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_pos[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);
	
	retValue = canSend(handle, &cmsg, &len);

	EnterCriticalSection(&cSec);
	if(pos > 0){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);

	delete[] param_pos;
	
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS_TIME_REL, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}

	return retValue;
}
NTCAN_RESULT ARM::CMD_MOV_POS_TIME_REL(int index, float pos, float vel, float acc, float cur, float time){
	this->checkStatus(index);
	vel = this->checkVel(index,vel);
	acc = this->checkAcc(index,acc);
	cur = this->checkCur(index,cur);

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[4];
	int32_t len=4;
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_acc=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];
	uint8_t *param_time=(uint8_t*)new uint8_t[4];

	float2Bit(pos, param_pos);
	float2Bit(vel, param_vel);
	float2Bit(acc, param_acc);
	float2Bit(cur, param_cur);
	float2Bit(time, param_time);

	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
		cmsg[2].data[i] = 0;
		cmsg[3].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;
	cmsg[2].id = module[index].id|MSG_MASTER;
	cmsg[3].id = module[index].id|MSG_MASTER;
	cmsg[0].len=8;
	cmsg[1].len=8;
	cmsg[2].len=8;
	cmsg[3].len=3;

	cmsg[0].data[0]=0x15; // length of each message data
	cmsg[1].data[0]=0x0F;
	cmsg[2].data[0]=0x09;
	cmsg[3].data[0]=0x03;

	cmsg[0].data[1]=0x84; // fragmentation
	cmsg[1].data[1]=0x85;
	cmsg[2].data[1]=0x85;
	cmsg[3].data[1]=0x86;

	cmsg[0].data[2]=_MOV_POS_TIME_REL; // command

	for (int i=3;i<7;i++)
		cmsg[0].data[i]=param_pos[i-3];

	cmsg[0].data[7]=param_vel[0];
	for (int i=2;i<5;i++)
		cmsg[1].data[i]=param_vel[i-1];
	
	for (int i=5;i<8;i++)
		cmsg[1].data[i]=param_acc[i-5];
	cmsg[2].data[2]=param_acc[3];

	for(int i=3;i<7;i++)
		cmsg[2].data[i]=param_cur[i-3];

	cmsg[2].data[7]=param_time[0];
	for(int i=2;i<5;i++)
		cmsg[3].data[i]=param_time[i-1];
			
	retValue = canSend(handle, cmsg, &len);

	EnterCriticalSection(&cSec);
	if(pos > 0){
		module[index].movForth = true;
		module[index].movBack = false;
	}
	else{
		module[index].movForth = false;
		module[index].movBack = true;
	}			
	LeaveCriticalSection(&cSec);

	delete[] cmsg;
	delete[] param_pos;
	delete[] param_vel;
	delete[] param_acc;
	delete[] param_cur;
	delete[] param_time;
	
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS_TIME_REL, ERROR code is "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;	
}
NTCAN_RESULT ARM::CMD_MOV_POS_LOOP(int index, float pos){
	return 0; 
}
NTCAN_RESULT ARM::CMD_MOV_POS_LOOP(int index, float pos ,float vel, float acc, float cur){
	return 0; 
}
NTCAN_RESULT ARM::CMD_MOV_CUR(int index, float cur){
	//check pos ?
	this->checkStatus(index);
	cur = this->checkCur(index,cur);

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];

	float2Bit(cur, param_cur);
	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_CUR;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_cur[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_cur;
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_MOV_CUR, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue;	
}    
NTCAN_RESULT ARM::CMD_MOV_VEL(int index, float vel){
	// need to be set time of stopping; 
	this->checkStatus(index); 
	vel = this->checkVel(index,vel);
	

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];

	float2Bit(vel, param_vel);
	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_VEL;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_vel[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);
	delete[] param_vel;

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_MOV_VEL, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue;
}
NTCAN_RESULT ARM::CMD_MOV_VEL(int index, float vel, float cur){
	this->checkStatus(index);
	vel = this->checkVel(index,vel);
	cur = this->checkCur(index,cur);

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[2];
	int32_t len=2;
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];

	float2Bit(vel, param_vel);
	float2Bit(cur, param_cur);
	
	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
	}

	cmsg[0].id = module[index].id|MSG_MASTER;
	cmsg[1].id = module[index].id|MSG_MASTER;
	cmsg[0].len=8;
	cmsg[1].len=5;

	cmsg[0].data[0]=0x09; // length of each message data
	cmsg[1].data[0]=0x03;
	
	cmsg[0].data[1]=0x84; // fragmentation
	cmsg[1].data[1]=0x86;
	
	cmsg[0].data[2]=_MOV_VEL; // command

	for (int i=3;i<7;i++)
		cmsg[0].data[i]=param_vel[i-3];

	cmsg[0].data[7]=param_cur[0];
	for (int i=2;i<5;i++)
		cmsg[1].data[i]=param_cur[i-1];	
	
	retValue = canSend(handle, cmsg, &len);	
	delete[] cmsg;
	delete[] param_cur;
	delete[] param_vel;
	
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_MOV_VEL, Error code: " << retValue<<endl;
		return retValue;
	}
	return retValue;
}
NTCAN_RESULT ARM::CMD_MOV_GRIP(int index, float cur){

	this->checkStatus(index);
	cur = this->checkCur(index,cur);

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];

	float2Bit(cur, param_cur);
	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_MOV_GRIP;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_cur[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);
	delete[] param_cur;

	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_MOV_GRIP, Error code: " << retValue<<endl;
		return retValue;
	}
	return retValue;
}   

NTCAN_RESULT ARM::SET_TARGET_VEL(int index, float val){
	val = this->checkVel(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_val=(uint8_t*)new uint8_t[4];
	float2Bit(val, param_val);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_SET_TARGET_VEL;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_val[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_val;
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_VEL, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}   
NTCAN_RESULT ARM::SET_TARGET_ACC(int index, float val){
	// this->checkStatus(index); 
	val = this->checkAcc(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_val=(uint8_t*)new uint8_t[4];
	float2Bit(val, param_val);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_SET_TARGET_ACC;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_val[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_val;
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_ACC, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}	 
NTCAN_RESULT ARM::SET_TARGET_JER(int index, float val){

	// this->checkStatus(index); 
	val = this->checkJer(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_val=(uint8_t*)new uint8_t[4];
	float2Bit(val, param_val);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_SET_TARGET_JER;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_val[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_val;
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_JER, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}  
NTCAN_RESULT ARM::SET_TARGET_CUR(int index, float val){

	// this->checkStatus(index); 
	val = this->checkCur(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_val=(uint8_t*)new uint8_t[4];
	float2Bit(val, param_val);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_SET_TARGET_CUR;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_val[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_val;
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_CUR, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}  
NTCAN_RESULT ARM::SET_TARGET_TIME(int index, float val){

	// this->checkStatus(index); 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_val=(uint8_t*)new uint8_t[4];
	float2Bit(val, param_val);

	cmsg.len=6;
	cmsg.id=module[index].id|MSG_MASTER;

	for (int i=0; i<8; i++)
		cmsg.data[i]=0;
	cmsg.data[0]=0x05;
	cmsg.data[1]=_SET_TARGET_TIME;
	for (int i=2;i<6;i++)
		cmsg.data[i]=param_val[i-2];

	// retValue = canWrite(handle, &cmsg, &len, NULL);	
	retValue = canSend(handle, &cmsg, &len);

	delete[] param_val;
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_TIME, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}  


NTCAN_RESULT ARM::SET_TARGET_POS(int index, float val){
	// new version
	return 0; 
}

NTCAN_RESULT ARM::SET_TARGET_POS_REL(int index, float val){
	// new version
	return 0; 
}
void ARM::float2Bit(float float_num, uint8_t* bit_num){
	union { 
		uint8_t n[4]; 
		float num;
	}tmp;
	tmp.num = float_num;
	for(int i=0; i<4; i++)
		bit_num[i]=tmp.n[i];
}
float ARM::bit2Float(uint8_t* bit_num){
	union { 
		uint8_t n[4]; 
		float num; 
	}tmp;
	for(int i=0; i<4; i++ )
		tmp.n[i] = bit_num[i];
	return tmp.num;
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
float ARM::checkJer(int index, float jer){
	return 0; 
}; 

// error in func. 
float ARM::checkTime(int index, float time, float posTar){
	float posNow = module[index].position; 
	float velNow = module[index].velocity; 
	float minTime, resetTime; 
	minTime = (posTar-posNow)/MaxVel[index]; 
	resetTime = (posTar-posNow)/velNow; 
	if (time<minTime)
	{
		cout<<"Module"<< index << ":  minimun time is "<< minTime<< "; reset time to "<< resetTime<<endl; 
	}
	return time; 
}
float ARM::checkTimeRel(int index, float time, float posTar){
	// int posNow = module[index].position; 
	float velNow = module[index].velocity; 
	float minTime, resetTime; 
	minTime = posTar/MaxVel[index]; 
	resetTime = posTar/velNow; 
	if (time<minTime)
	{
		cout<<"Module"<< index << ":  minimun time is "<< minTime<< "; reset time to "<< resetTime<<endl; 
	}
	return time; 
}


void ARM::canPolling(){
	bool comm = false;
	bool poll = false;

	// use synchronization to check the shared variables
	EnterCriticalSection(&cSec);
	if (canCommActive == true)
		comm = true;
	else
		comm = false;
	if (canPollActive == true)
		poll = true;
	else
		poll = false;
	LeaveCriticalSection(&cSec);

	while(poll){
		while(comm){
			// functions when communicate
			canMsgPoll();
			// judge whether communicate
			// use synchronization to check shared variable
			EnterCriticalSection(&cSec);
			if (canCommActive == true)
				comm = true;
			else
				comm = false;
			LeaveCriticalSection(&cSec);		
		}
		// use synchronization to check shared variable
		EnterCriticalSection(&cSec);
		if (canPollActive == true)
			poll = true;
		else
			poll = false;
		LeaveCriticalSection(&cSec);		
	}
}
int ARM::canMsgPoll(){
	NTCAN_RESULT retValue;
	int32_t len = 40;
	int parseRet = 0;
	
	
	int count=0;

	retValue = canTake(handle, pollBuf, &len); //non-block reading

	if(retValue != NTCAN_SUCCESS)
		return RX_TIME_OUT;

	if(len == 0)
		return EMPTY_Q;
	//start = clock();
	while(count < len){		
		parseRet = procNextMsg(count, pollBuf); 
		count++;
	}
	//end = clock();
	//timeSum = timeSum + (float)(end - start) / (float)(CLOCKS_PER_SEC * len);
	//parseCounter++;
	return parseRet;

}
int ARM::procNextMsg(int count, CMSG* cmsg){
	// count ?
	// more handle to be needed 
	int mIndex = 0;
	int parseRet = 0;
	if(cmsg[count].data[1] == _FRAG_BEGIN || cmsg[count].data[1] == _FRAG_MIDDLE || cmsg[count].data[1] == _FRAG_END){
		// if it is a fragment message, push into the buffer
		mIndex = cmsg[count].id & GET_ID - OFFSET;
		// copy the msg into the buffer
		fragBuf[mIndex][fragLen[mIndex]].id = cmsg[count].id;
		fragBuf[mIndex][fragLen[mIndex]].len = cmsg[count].len;
		for(int i = 0; i < 8; i++)
			fragBuf[mIndex][fragLen[mIndex]].data[i] = cmsg[count].data[i];
		fragLen[mIndex]++;

		if(cmsg[count].data[1] == _FRAG_END){
			// process fragment here, potentially seperate to another process
			parseRet = parseFragMsg(mIndex);        //  PARSE
			fragLen[mIndex] = 0;
			return parseRet;
		}
		return FRAGGING;

	}
	else{
		// non fragmented messages
		mIndex = cmsg[count].id & GET_ID - OFFSET;
		if(cmsg[count].data[1] == _POS_REACHED){ // cmd pos reached
			// position parsed
			module[mIndex].posReached = true; // maybe we need inter thread control here
			module[mIndex].moving = false;
			pos_buffer[0] = cmsg[count].data[2];
			pos_buffer[1] = cmsg[count].data[3];
			pos_buffer[2] = cmsg[count].data[4];
			pos_buffer[3] = cmsg[count].data[5];
			module[mIndex].position = bit2Float(pos_buffer);

			EnterCriticalSection(&cSec);
			module[mIndex].mEvent = EVENT_REACHED;
			LeaveCriticalSection(&cSec);
			// if(module[mIndex].hold == true){
				
			// }
		}
		else if(cmsg[count].data[1] == _MOV_END){
			// response from slave is position (standing still)
			module[mIndex].moveEnd = true; // maybe we need inter thread control here
			module[mIndex].moving = false;
			pos_buffer[0] = cmsg[count].data[2];
			pos_buffer[1] = cmsg[count].data[3];
			pos_buffer[2] = cmsg[count].data[4];
			pos_buffer[3] = cmsg[count].data[5];
			module[mIndex].position = bit2Float(pos_buffer);

			//EnterCriticalSection(&cSec);
			//module[mIndex].mEvent = EVENT_REACHED;
			//LeaveCriticalSection(&cSec);

		}
		else if((cmsg[count].data[1] == _MOV_POS)||(cmsg[count].data[1] == _MOV_POS_REL)||cmsg[count].data[1] == (_MOV_POS_TIME)||(cmsg[count].data[1] == _MOV_POS_TIME_REL)){
			//response from Slave is estimated time 
			//.movBack & .movForth have changed in CMD_POS
			module[mIndex].brake = false;
			module[mIndex].moving = true;

		}
		else if(cmsg[count].data[1] == _MOV_VEL){
			// response from Slave is "OK". 
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				// CONDITION: vel in CMD_MOV_VEL == 0 or not ??? 
				//module[mIndex].brake = false;
				//module[mIndex].moving = true;
			}
		}
		else if (cmsg[count].data[1] == _MOV_CUR){
			// response from Slave is "OK". 
		}
		else if((cmsg[count].data[1] == _CMD_STOP)||(cmsg[count].data[1] == _CMD_EMERGENCY_STOP)){
			// response from Slave is "OK" (if SUCCESS)
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module[mIndex].brake = true;
				module[mIndex].moving = false;
				module[mIndex].movBack = false; 
				module[mIndex].movForth = false; 
			}
			else
				return CMD_STOP_FAIL;
		}
		else if(cmsg[count].data[1] == _CMD_ERROR){
			module[mIndex].error = true;
			module[mIndex].errorCode = cmsg[count].data[2];
			// error dealt in the event handler
		}
		else if(cmsg[count].data[1] == _CMD_INFO){
			// deal with error here
		}
		else if(cmsg[count].data[1] == _CMD_ACK){
			// response from Slave is "OK" (If all errors acknowledged)
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module[mIndex].error = false;
			}
			else
				return ACK_FAIL;
		}
		else if(cmsg[count].data[1] == _CMD_REFERENCE){
			// response from Slave is "OK" (If SUCCESS)
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module[mIndex].brake = false;
				module[mIndex].moving = true;
				module[mIndex].referenced = true; 
			}
			else
				return REFERENCE_FAIL;
		}
		else if (cmsg[count].data[1] == _SET_TARGET_VEL || cmsg[count].data[1] == _SET_TARGET_ACC || cmsg[count].data[1] == _SET_TARGET_JER || cmsg[count].data[1] == _SET_TARGET_CUR || cmsg[count].data[1] == _SET_TARGET_TIME){
			// response from Slave is "OK" (if SUCCESS)
		}
		else{
			// cout<<(int)cmsg[count].data[1]<<endl;
			return UNKNOWN_MSG;
		}
		return PARSE_SUCCESS;
	}
}


int ARM::parseFragMsg(int mIndex){
	if(fragLen[mIndex]==3 && fragBuf[mIndex][0].data[2] == _GET_STATE){
		pos_buffer[0] = fragBuf[mIndex][0].data[3];
		pos_buffer[1] = fragBuf[mIndex][0].data[4];
		pos_buffer[2] = fragBuf[mIndex][0].data[5];
		pos_buffer[3] = fragBuf[mIndex][0].data[6];
		module[mIndex].position = bit2Float(pos_buffer);

		vel_buffer[0] = fragBuf[mIndex][0].data[7];
		vel_buffer[1] = fragBuf[mIndex][1].data[2];
		vel_buffer[2] = fragBuf[mIndex][1].data[3];
		vel_buffer[3] = fragBuf[mIndex][1].data[4];
		module[mIndex].velocity = bit2Float(vel_buffer);
			
		cur_buffer[0] = fragBuf[mIndex][1].data[5];
		cur_buffer[1] = fragBuf[mIndex][1].data[6];
		cur_buffer[2] = fragBuf[mIndex][1].data[7];
		cur_buffer[3] = fragBuf[mIndex][2].data[2];
		module[mIndex].current = bit2Float(cur_buffer);

		// the Byte of states
		// module[mIndex].status = fragBuf[mIndex][2].data[3]; 

		module[mIndex].referenced = ((fragBuf[mIndex][2].data[3]&CHECK_REFERENCED) == 0) ? false : true;
		module[mIndex].moving = ((fragBuf[mIndex][2].data[3]&CHECK_MOVING) == 0) ? false : true;
		module[mIndex].brake = ((fragBuf[mIndex][2].data[3]&CHECK_BRAKE) == 0) ? false : true;
		module[mIndex].moveEnd = ((fragBuf[mIndex][2].data[3]&CHECK_MOV_END) == 0) ? false : true;
		module[mIndex].posReached = ((fragBuf[mIndex][2].data[3]&CHECK_POS_REACHED) == 0) ? false : true;
		module[mIndex].error = ((fragBuf[mIndex][2].data[3]&CHECK_ERROR) == 0) ? false : true;
		module[mIndex].errorCode = fragBuf[mIndex][2].data[4]; 
		

		return PARSE_SUCCESS;
	}
	else if(fragBuf[mIndex][0].data[2] == _GET_CONFIG){
			// do nothing here, just show we know this message
		return PARSE_SUCCESS;
	}
	else{
		return UNKNOWN_MSG;
	}
}

void ARM::eventHandling(){
	bool eHandle = false;
	int eventState = 0;
	int index = 0;
	EnterCriticalSection(&cSec);
	if (eventActive == true)
		eHandle = true;
	else
		eHandle = false;
	LeaveCriticalSection(&cSec);

	while(eHandle){
		for(int i = 8; i <= 15; i ++){
			index = i - OFFSET;
			EnterCriticalSection(&cSec);
			eventState = module[index].mEvent;
			LeaveCriticalSection(&cSec);

			if (eventState == EVENT_REACHED){

				// callBackReached(index);  // 
				callBackReachedNotBrake(index); 

				EnterCriticalSection(&cSec);
				module[index].mEvent = EVENT_NULL;
				LeaveCriticalSection(&cSec);
			}
		}

		EnterCriticalSection(&cSec);
		if (eventActive == true)
			eHandle = true;
		else
			eHandle = false;
		LeaveCriticalSection(&cSec);
	}
}
void ARM::errorHandler(int index){
	uint8_t ErrCode; 
	ErrCode = module[index].errorCode; 
	cout<<"Module "<< module[index].id << " encounters Error "<< ErrCode <<endl; 
	switch(ErrCode)
	{
	case	INFO_BOOT:
		cout<<"Error codes: 0x0001(INFO). Module is successfully booted !"<<endl;
		break;
	case	INFO_NO_RIGHTS:
		cout<<"Error code: 0x03(INFO). No rights to cmd !"<<endl;
		break;
	case	INFO_UNKNOWN_COMMAND:
		cout<<"Error code: 0x04(INFO). Unknown command !"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	INFO_FAILED:
		cout<<"Error code: 0x05. Command failed!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	INFO_NOT_REFERENCED:     // not referenced
		cout<<"Error code: 0x06(INFO). Not referenced!"<<endl;
		CMD_REFERENCE(index); 
		break;
	case	INFO_SEARCH_SINE_VECTOR:
		cout<<"Error code: 0x0007(INFO). Search sine vector!"<<endl; 
		break; 
	case	INFO_NO_ERRORS:
		cout<<"Error code: 0x0008(INFO). No Errors!"<<endl;
		break;
	case	INFO_COMMUNICATE_ERROR:
		cout<<"Error code: 0x09. Error in communication !"<<endl;
		canDisconnect(); 
		canInitial(); 
		//CMD_REFERENCED(index); 
		//canConnect();
		//canScanBus();
		break;
	case	INFO_TIMEOUT:
		cout<<"Error code: 0x10. Timeout in communication!"<<endl;
		break;
	case	INFO_WRONG_BAUDRATE:
		cout<<"Error code: 0x16. Wrong Baudrate!"<<endl;
		// Default is BAUD_1000. 
		// canSetBaudrate(handle, BAUDRATE);
		break;
	case	INFO_CHECKSUM:
		// CRC error
		cout<<"Error code: 0x19. Checksum is incorrect "<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	INFO_MESSAGE_LENGTH:
		cout<<"Error code: 0x1D. D-Len does not match."<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	INFO_WRONG_PARAMETER:
		cout<<"Error code: 0x1E. Wrong parameter!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_WRONG_RAMP_TYPE:
		cout<<"Error code: 0xC8. No valid motion profile!"<<endl;;
		CMD_STOP(index); 
		module[index].movBack = false;
		module[index].movForth = false;
		// canDisconnect();
		// exit(0);
		break;
	case	ERROR_CONFIG_MEMORY:
		cout<<"Error code: 0xD2. Configuration range is incorrect!"<<endl;
		CMD_STOP(index); 
		module[index].movBack = false;
		module[index].movForth = false;
		// canDisconnect();
		// exit(0);
		break;
	case	ERROR_SOFT_LOW:
		cout<<"Error code: 0xD5. Module exceeded software low limit!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_SOFT_HIGH:
		cout<<"Error code: 0xD6. Module exceeded software high limit!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_EMERGENCY_STOP:
		cout<<"Error code: 0xD9. Emergency Stop!"<<endl;
		CMD_REFERENCE(index); 
		break;
	case	ERROR_TOW:           // A common err.
		cout<<"Error code: 0xDA. Towing Error! Reducing load of module!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_TOO_FAST:
		cout<<"Error code: 0xE4. Too Fast. Reducing current of module!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_COMMUTATION: 
		cout<<"Error code: 0xDD. Module fails to commutate."<<endl; 
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break; 
	case	ERROR_FRAGMENTATION:
		cout<<"Error code: 0xDC. Error in fragmentation. Data packets lost!"<<endl;
		break;
	case	ERROR_CURRENT:
		cout<<"Error code: 0xDE. Current too large! Reducing load of module!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_I2T:
		cout<<"Error code: 0xDF. I2T Error!  Reducing load of module!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		break;
	case	ERROR_INITIALIZE:
		cout<<"Error code: 0xE0. Module could not be initialized"<<endl;
		canDisconnect();
		break;
	case	ERROR_TEMP_LOW:
		cout<<"Error code: 0x70. Temprature of module is too low!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		canDisconnect();
		exit(0);
		break;
	case	ERROR_TEMP_HIGH:
		cout<<"Error code: 0x71. Temprature of module is too high!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		canDisconnect();
		exit(0);
		break;
	case	ERROR_LOGIC_LOW:
		cout<<"Error code: 0x72. Logic voltage is too low!"<<endl;
		canDisconnect();
		exit(0);
		break;
	case	ERROR_LOGIC_HIGH:
		cout<<"Error code: 0x73. Logic voltage is too high!"<<endl;		
		canDisconnect();
		exit(0);
		break;
	case	ERROR_MOTOR_V_LOW:
		cout<<"Error code: 0x74. Motor voltage is too low!"<<endl;
		canDisconnect();
		exit(0);
		break;
	case	ERROR_MOTOR_V_HIGH:
		cout<<"Error code: 0x75. Motor voltage is too high!"<<endl;
		canDisconnect();
		exit(0);
		break;
	case	ERROR_CABLE_BREAK:
		cout<<"Error code: 0x76. Cable is defective!"<<endl;
		CMD_STOP(index);
		module[index].movBack = false;
		module[index].movForth = false;
		// canDisconnect();
		exit(0);
		break;
	default: 
		cout<<"Error not defined yet"<<endl; 
		CMD_STOP(index); 
		module[index].movBack = false;
		module[index].movForth = false;
	}
	CMD_ACK(index);
	}
void ARM::callBackReached(int index){
	// hold	
	bool flagForth = false;
	bool flagBack = false;
	if(module[index].error == true){
		errorHandler(index);
	}
	// MOVE_VEL(index, 0.0);
	
	// v3 = (float)(rand() % 20001) / 10000.0 + 8.0;
	// v3 should range from 4 to 12 
	// v3 =6.0;
	// v3 = (float)(rand() % 76000) / 10000.0 + 4.2;
	// v3 = (float)(rand() % 76) / 10.0 + 4.2;
	// v3 = 6.0; //5.0; //6.5;
	// v0 = 4.0; //5.0;

	// v0 = 2.5;
	v3 = 1.5;
	v5 = 2.1;
	v6 = 5.0; 
	// v7 = 5.5;
	// v3 = 4.0;
	// v5 = 2.0;
	// v7 = 6.0;

	EnterCriticalSection(&cSec);
	flagBack = module[index].movBack;
	flagForth = module[index].movForth;
	LeaveCriticalSection(&cSec);


	/*if(index == 0){
		if(module[index].movBack == true){
			MOVE_POS(index, -20.0, v0, 20.0, 10.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = false;
			module[index].movForth = true;	
			LeaveCriticalSection(&cSec);
		}
		else if(module[index].movForth == true){
			MOVE_POS(index, -40.0, v0, 20.0, 10.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = true;
			module[index].movForth = false;	
			LeaveCriticalSection(&cSec);
		}
	}*/

	if(index ==3){
		if(flagBack == true){
			CMD_MOV_POS(index, 80.0, v3, 20.0, 6.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = false;
			module[index].movForth = true;
			LeaveCriticalSection(&cSec);
		}
		else if(flagForth == true){
			CMD_MOV_POS(index, 60.0, v3, 20.0, 6.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = true;
			module[index].movForth = false;
			LeaveCriticalSection(&cSec);
		}
	}

	if(index == 5){
		if(module[index].movBack == true){
			CMD_MOV_POS(index, 90.0, v5, 20.0, 4.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = false;
			module[index].movForth = true;	
			LeaveCriticalSection(&cSec);
		}
		else if(module[index].movForth == true){
			CMD_MOV_POS(index, 70.0, v5, 20.0, 4.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = true;
			module[index].movForth = false;	
			LeaveCriticalSection(&cSec);
		}
	}
	
	// To module ID 14
	if(index == 6){
		if(module[index].movBack == true){
			CMD_MOV_POS(index, 15.0, v6, 5.0, 4.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = false;
			module[index].movForth = true;	
			LeaveCriticalSection(&cSec);
		}
		else if(module[index].movForth == true){
			CMD_MOV_POS(index, -15.0, v6, 5.0, 4.0);

			EnterCriticalSection(&cSec);
			module[index].movBack = true;
			module[index].movForth = false;	
			LeaveCriticalSection(&cSec);
		}
	}

	/*if(index == 7){
		if(module[index].movBack == true){
			MOVE_POS(index, 50.0, v7, 20.0, 1.8);

			EnterCriticalSection(&cSec);
			module[index].movBack = false;
			module[index].movForth = true;	
			LeaveCriticalSection(&cSec);
		}
		else if(module[index].movForth == true){
			MOVE_POS(index, 10.0, v7, 20.0, 1.8);

			EnterCriticalSection(&cSec);
			module[index].movBack = true;
			module[index].movForth = false;	
			LeaveCriticalSection(&cSec);
		}
	}*/

	
}



void ARM::callBackReachedNotBrake(int index){
	int flagmEvent = 0; 
	bool flagMoving; 

	int flagCycle; 
	clock_t timeStart, timeFinish; 
	clock_t timeLimit = 10; 
	if(module[index].error == true){
		errorHandler(index);
	}
	flagCycle = 1; 
	timeStart = clock(); 
	while(flagCycle)
	{
		timeFinish = clock(); 
		if((timeFinish-timeStart)/(clock_t)1000 <=timeLimit)
		{
			EnterCriticalSection(&cSec); 
			flagMoving = module[index].moving; 
			flagmEvent = module[index].mEvent; 
			if (flagmEvent == EVENT_REACHED && flagMoving == false)
			{
				module[index].movBack = false; 
				module[index].movForth = false; 
				CMD_MOV_VEL(index, 0);  // not brake; 
				flagCycle = 0;
			}
			LeaveCriticalSection(&cSec); 
		}
	}	
}

void ARM::enableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = true;			
	LeaveCriticalSection(&cSec);
}
void ARM::disableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = false;
	LeaveCriticalSection(&cSec);
}
void ARM::enableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = true;
	LeaveCriticalSection(&cSec);
}
void ARM::disableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = false;
	LeaveCriticalSection(&cSec);
}
void ARM::enableEvent(){
	EnterCriticalSection(&cSec);
	eventActive = true;
	LeaveCriticalSection(&cSec);
}
void ARM::disableEvent(){
	EnterCriticalSection(&cSec);
	eventActive = false;
	LeaveCriticalSection(&cSec);
}

void ARM::openThread(){
	this->enableCanPoll(); 
	this->enableCanComm(); 
	this->pThreadPolling = (HANDLE)_beginthreadex(NULL, 0, ARM::canPollingThreadStartUp, this, CREATE_SUSPENDED, NULL); 
	ResumeThread(pThreadPolling); 
	this->pThreadHandling = (HANDLE)_beginthreadex(NULL, 0, ARM::eventThreadStartUp, this, CREATE_SUSPENDED, NULL); 
	ResumeThread(pThreadHandling); 

}

void ARM::closeThread(){
	this->disableCanPoll();
	this->disableCanComm();  
	TerminateThread(pThreadPolling, 0); 
	TerminateThread(pThreadHandling, 0); 
	ExitProcess(0); 
}