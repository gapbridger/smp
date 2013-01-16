#include <iostream>
#include "Dkernel.h"
KERNEL::KERNEL(){
	/*
	1. Open thread
	2. Module initialization 
	*/
	this->threadOpen(); 

}
KERNEL::~KERNEL(){
	this->threadClose(); 
}

void KERNEL::threadOpen(){
	this->enableCanPoll(); 
	this->enableCanComm(); 
	this->pThreadPolling = (HANDLE)_beginthreadex(NULL, 0, KERNEL::canPollingThreadStartUp, this, CREATE_SUSPENDED, NULL); 
	ResumeThread(pThreadPolling); 
	this->pThreadHandling = (HANDLE)_beginthreadex(NULL, 0, KERNEL::eventThreadStartUp, this, CREATE_SUSPENDED, NULL); 
	ResumeThread(pThreadHandling); 
}
void KERNEL::threadClose()
{
	this->disableCanPoll();
	this->disableCanComm();  
	TerminateThread(pThreadPolling, 0); 
	TerminateThread(pThreadHandling, 0); 
	ExitProcess(0); 
}

void KERNEL::enableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = true;			
	LeaveCriticalSection(&cSec);
}
void KERNEL::disableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = false;
	LeaveCriticalSection(&cSec);
}
void KERNEL::enableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = true;
	LeaveCriticalSection(&cSec);
}
void KERNEL::disableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = false;
	LeaveCriticalSection(&cSec);
}

NTCAN_RESULT KERNEL::canPolling(){
	NTCAN_RESULT retValue; 
	int32_t len = 40; 
	int parseRet = 0; 
	int count = 0; 
	while(canCommActive)
	{
		retValue = canTake(handle, pollBuf, &len); 
		if(retValue != NTCAN_SUCCESS)
			return RX_TIME_OUT; 
		if(len == 0)
			return EMPTY_Q; 
		while(count<len)
		{
			parseRet = procNextMsg(count, pollBuf);
			count++; 
		}
		return parseRet; 
	}
}

int KERNEL::procNextMsg(int count, CMSG* cmsg){
	// update 
	int mIndex = 0;
	int parseRet = 0;
	if(cmsg[count].data[1] == _FRAG_BEGIN || cmsg[count].data[1] == _FRAG_MIDDLE || cmsg[count].data[1] == _FRAG_END){
		// if it is a fragment message, push into the buffer
		mIndex = cmsg[count].id & GET_ID - OFFSET;    // ?
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

int KERNEL::parseFragMsg(int mIndex){
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
	else if(fragLen[mIndex]==2 && fragBuf[mIndex][0].data[2] == _GET_STATE){
	}
	else if(fragLen[mIndex]==1 && fragBuf[mIndex][0].data[2] == _GET_STATE){
	}

	else if(fragBuf[mIndex][0].data[2] == _GET_CONFIG){
			// do nothing here, just show we know this message
		return PARSE_SUCCESS;
	}
	else{
		return UNKNOWN_MSG;
	}
}

NTCAN_RESULT KERNEL::CMD_GET_STATE(int index, float time, uint8_t mode){
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

NTCAN_RESULT KERNEL::CMD_REFERENCE(int index){
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

NTCAN_RESULT KERNEL::CMD_STOP(int index){

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

NTCAN_RESULT KERNEL::CMD_EMERGENCY_STOP(int index){

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
NTCAN_RESULT KERNEL::CMD_MOV_POS(int index, float pos ,float vel, float acc, float cur){
	// parameter: index, pos, {vel, acc, cur} 
	// this->checkStatus(index); 
	// this->checkPos(index, pos); 
	// vel = this->checkVel(index, vel); 
	// acc = this->checkAcc(index, acc); 
	// cur = this->checkCur(index, cur);

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

NTCAN_RESULT KERNEL::CMD_MOV_VEL(int index, float vel, float cur){
	// this->checkStatus(index);
	// vel = this->checkVel(index,vel);
	// cur = this->checkCur(index,cur);

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

NTCAN_RESULT KERNEL::SET_TARGET_VEL(int index, float val){
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
NTCAN_RESULT KERNEL::SET_TARGET_POS(int index, float val){
	val = this->checkPos(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg; 
	int32_t len = 1; 
	uint8_t *param_val(uint8_t*)new uint8_t[4]; 
	float2Bit(val, param_val); 

	cmsg.len=6; 
	cmsg.id=module[index].id|MSG_MASTER; 

	for(int i=0; i<8; i++)
		cmsg.data[i] = 0; 
	cmsg.data[0] = 0x05; 
	cmsg.data[1] = _SET_TARGET_VEL; 
	for(int i=2; i<6; i++)
		cmsg.data[i] = param_val[i-2]; 
	retValue = canSend(handle, &cmsg, &len); 
	delete[] param_val; 
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_POS, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}
NTCAN_RESULT KERNEL::SET_TARGET_POS_REL(int index, float val){
	val = this->checkRelPos(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg; 
	int32_t len = 1; 
	uint8_t *param_val(uint8_t*)new uint8_t[4]; 
	float2Bit(val, param_val); 

	cmsg.len=6; 
	cmsg.id=module[index].id|MSG_MASTER; 

	for(int i=0; i<8; i++)
		cmsg.data[i] = 0; 
	cmsg.data[0] = 0x05; 
	cmsg.data[1] = _SET_TARGET_VEL; 
	for(int i=2; i<6; i++)
		cmsg.data[i] = param_val[i-2]; 
	retValue = canSend(handle, &cmsg, &len); 
	delete[] param_val; 
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_SET_TARGET_POS_REL, Error code is " << retValue<<endl;
		return retValue;
	}
	return retValue; 
}

void KERNEL::float2Bit(float float_num, uint8_t* bit_num){
	union { 
		uint8_t n[4]; 
		float num;
	}tmp;
	tmp.num = float_num;
	for(int i=0; i<4; i++)
		bit_num[i]=tmp.n[i];
}
float KERNEL::bit2Float(uint8_t* bit_num){
	union { 
		uint8_t n[4]; 
		float num; 
	}tmp;
	for(int i=0; i<4; i++ )
		tmp.n[i] = bit_num[i];
	return tmp.num;
}