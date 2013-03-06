#include <iostream>
#include "SMP.h"
using namespace std; 

MODULE module[MODULE_NUM];    // externals initialization

/*****************
Schunk Motion Protocol
1. CAN-bus
2. polling & handling threads
3. all independent modules included in CAN-communication
4. cmd lib.

******************/

SMP::SMP(){
	uint8_t status_init = 0xE1, error_init = 0x00, mode_init = 0x07; 
	float pos_init = 0, vel_init = 0, cur_init = 0, acc_init = 0; 
	int idx=0; 
	for (int32_t i=ID_START; i<=ID_END; i++)
	{
		module[idx].setId(i);
		module[idx].setStatusCode(status_init);   // ref, moving, prog.mode, warning, err, brake, move.end, pos.reached 
		module[idx].setErrorCode(error_init);
		module[idx].setMode(mode_init); 
		module[idx].updatePos(pos_init); 
		module[idx].updateVel(vel_init); 
		module[idx].updateAcc(acc_init);
		module[idx].updateCur(cur_init); 
		idx++; 
	}

	handle = NULL; 
	baud = NTCAN_BAUD_1000; 
	
	for(int i = 0; i<8; i++)
		fragLen[i]=0; 

	pos_buffer=(uint8_t*)new uint8_t[4];
	vel_buffer=(uint8_t*)new uint8_t[4];
	cur_buffer=(uint8_t*)new uint8_t[4];
	
	InitializeCriticalSection(&cSec);
	canCommActive = false;
	canPollActive = true;
	eventActive = true;

	// this->threadOpen(); 
}

SMP::~SMP(){

	DeleteCriticalSection(&cSec);	
	// this->threadClose(); 
}




void SMP::canPolling(){
	// NTCAN_RESULT retValue = 0; 
	// int32_t len; 
	// int parseRet; 
	// int count; 
	while(canCommActive)
	{
		canMsgPoll();  
	}
	
}
int SMP::canMsgPoll(){
	NTCAN_RESULT retValue; 
	int32_t len = 40; 
	int parseRet = 0; 

	int count = 0; 

	retValue = canTake(handle, pollBuf, &len); 
	if(retValue != NTCAN_SUCCESS)
		return RX_TIME_OUT; 
	if(len == 0)
		return EMPTY_Q; 
	while(count<len){
		parseRet = procNextMsg(count, pollBuf);
		count++; 
	}
	return parseRet;
}

int SMP::procNextMsg(int count, CMSG* cmsg){
	// update 
	int mIndex = 0;
	int parseRet = 0;
	uint8_t module_status; 
	float module_pos = 0; 
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
			module_status = module[mIndex].getStatusCode();
			module_status = (module_status | TRUE_POS_REACHED & FALSE_MOVING); 
			module[mIndex].setStatusCode(module_status);  

			pos_buffer[0] = cmsg[count].data[2];
			pos_buffer[1] = cmsg[count].data[3];
			pos_buffer[2] = cmsg[count].data[4];
			pos_buffer[3] = cmsg[count].data[5];
			module_pos = bit2Float(pos_buffer);
			module[mIndex].updatePos(module_pos);
		}
		else if(cmsg[count].data[1] == _MOV_END){
			// response from slave is position (standing still)
			module_status = module[mIndex].getStatusCode();
			module_status = (module_status | TRUE_MOV_END & FALSE_MOVING); 
			module[mIndex].setStatusCode(module_status); 

			pos_buffer[0] = cmsg[count].data[2];
			pos_buffer[1] = cmsg[count].data[3];
			pos_buffer[2] = cmsg[count].data[4];
			pos_buffer[3] = cmsg[count].data[5];
			module_pos = bit2Float(pos_buffer);
			module[mIndex].updatePos(module_pos); 

		}
		else if((cmsg[count].data[1] == _MOV_POS)||(cmsg[count].data[1] == _MOV_POS_REL)||cmsg[count].data[1] == (_MOV_POS_TIME)||(cmsg[count].data[1] == _MOV_POS_TIME_REL)){
			//response from Slave is estimated time 
			module_status = module[mIndex].getStatusCode();
			module_status = (module_status | TRUE_MOVING & FALSE_BRAKE); 
			module[mIndex].setStatusCode(module_status); 
		}
		else if(cmsg[count].data[1] == _MOV_VEL){
			// response from Slave is "OK". 
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				// CONDITION: vel in CMD_MOV_VEL == 0 or not ??? 
				//module[mIndex].brake = false;
				//module[mIndex].moving = true;
				module_status = module[mIndex].getStatusCode();
				module_status = (module_status | TRUE_MOVING & FALSE_BRAKE); 
				module[mIndex].setStatusCode(module_status); 
			}
		}
		else if (cmsg[count].data[1] == _MOV_CUR){
			// response from Slave is "OK". 
		}
		else if((cmsg[count].data[1] == _CMD_STOP)||(cmsg[count].data[1] == _CMD_EMERGENCY_STOP)){
			// response from Slave is "OK" (if SUCCESS)
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module_status = module[mIndex].getStatusCode();
				module_status = (module_status | TRUE_BRAKE & FALSE_MOVING); 
				module[mIndex].setStatusCode(module_status); 
			}
			else
				return CMD_STOP_FAIL;
		}
		else if(cmsg[count].data[1] == _CMD_ERROR){
			module_status = module[mIndex].getStatusCode();
			module_status = (module_status | TRUE_ERROR); 
			module[mIndex].setStatusCode(module_status); 
			module[mIndex].setErrorCode(cmsg[count].data[2]);
			// error dealt in the event handler
		}
		else if(cmsg[count].data[1] == _CMD_INFO){
			// deal with error here
		}
		else if(cmsg[count].data[1] == _CMD_ACK){
			// response from Slave is "OK" (If all errors acknowledged)
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module_status = module[mIndex].getStatusCode();
				module_status = (module_status & FALSE_ERROR); 
				module[mIndex].setStatusCode(module_status); 
			}
			else
				return ACK_FAIL;
		}
		else if(cmsg[count].data[1] == _CMD_REFERENCE){
			// response from Slave is "OK" (If SUCCESS)
			if(cmsg[count].data[2] == 0x4F && cmsg[count].data[3] == 0x4B){
				module_status = module[mIndex].getStatusCode();
				module_status = (module_status | TRUE_MOVING | TRUE_REFERENCED & TRUE_BRAKE); 
				module[mIndex].setStatusCode(module_status); 
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

int SMP::parseFragMsg(int mIndex){
	// NTCAN_RESULT retValue; 
	float module_pos, module_vel, module_cur; 
	// uint8_t module_status; 
	// int32_t mIndex; 
	// mIndex = module[index].getId(); 
	if(fragLen[mIndex]==3 && fragBuf[mIndex][0].data[2] == _GET_STATE){
		pos_buffer[0] = fragBuf[mIndex][0].data[3];
		pos_buffer[1] = fragBuf[mIndex][0].data[4];
		pos_buffer[2] = fragBuf[mIndex][0].data[5];
		pos_buffer[3] = fragBuf[mIndex][0].data[6];
		module_pos = bit2Float(pos_buffer); 
		module[mIndex].updatePos(module_pos); 
		//module[mIndex].position = bit2Float(pos_buffer);

		vel_buffer[0] = fragBuf[mIndex][0].data[7];
		vel_buffer[1] = fragBuf[mIndex][1].data[2];
		vel_buffer[2] = fragBuf[mIndex][1].data[3];
		vel_buffer[3] = fragBuf[mIndex][1].data[4];
		module_vel = bit2Float(vel_buffer); 
		module[mIndex].updateVel(module_vel); 
		//module[mIndex].velocity = bit2Float(vel_buffer);
			
		cur_buffer[0] = fragBuf[mIndex][1].data[5];
		cur_buffer[1] = fragBuf[mIndex][1].data[6];
		cur_buffer[2] = fragBuf[mIndex][1].data[7];
		cur_buffer[3] = fragBuf[mIndex][2].data[2];
		module_cur = bit2Float(cur_buffer); 
		module[mIndex].updateCur(module_cur);
		//module[mIndex].current = bit2Float(cur_buffer);

		// the Byte of states
		// module[mIndex].status = fragBuf[mIndex][2].data[3]; 

		// module[mIndex].referenced = ((fragBuf[mIndex][2].data[3]&CHECK_REFERENCED) == 0) ? false : true;
		// module[mIndex].moving = ((fragBuf[mIndex][2].data[3]&CHECK_MOVING) == 0) ? false : true;
		// module[mIndex].brake = ((fragBuf[mIndex][2].data[3]&CHECK_BRAKE) == 0) ? false : true;
		// module[mIndex].moveEnd = ((fragBuf[mIndex][2].data[3]&CHECK_MOV_END) == 0) ? false : true;
		// module[mIndex].posReached = ((fragBuf[mIndex][2].data[3]&CHECK_POS_REACHED) == 0) ? false : true;
		// module[mIndex].error = ((fragBuf[mIndex][2].data[3]&CHECK_ERROR) == 0) ? false : true;
		// module[mIndex].errorCode = fragBuf[mIndex][2].data[4];
		module[mIndex].setStatusCode(fragBuf[mIndex][2].data[3]); 
		module[mIndex].setErrorCode(fragBuf[mIndex][2].data[4]); 
		return PARSE_SUCCESS;
	}
	else if(fragLen[mIndex]==2 && fragBuf[mIndex][0].data[2] == _GET_STATE){
		return PARSE_SUCCESS;
	}
	else if(fragLen[mIndex]==1 && fragBuf[mIndex][0].data[2] == _GET_STATE){
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


void SMP::eventHandling(){
	// bool eHandle = false;
	int eventState = 0;
	uint8_t module_status = 0, module_error = 0; 
	int idx;
	/*
	EnterCriticalSection(&cSec);
	if (eventActive == true)
		eHandle = true;
	else
		eHandle = false;
	LeaveCriticalSection(&cSec);
	*/
	while(eventActive){
		// error checking
		for(idx = 0; idx <= ID_END - OFFSET; idx++)
		{
			if(module[idx].checkError()==true)
				this->errorHandler(idx);
		}

	}
}

NTCAN_RESULT SMP::CMD_GET_STATE(int index, float time, uint8_t mode){
	// none-param : response with pos, vel, cur;  for once;  
	// time: response with vel, acc, cur; constantly;   
	// time, mode:  mode = 0x01(pos), 0x02(vel), 0x04(cur); constantly; 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_time = (uint8_t*)new uint8_t[4];
	
	int32_t module_id; 
	
	for (int i = 0; i < 8; i++)
			cmsg.data[i] = 0;
	module_id = module[index].getId(); 
	cmsg.id = module_id|MSG_MASTER_TO_SLAVE;

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
		
		module[index].setMode(mode); 
		// module[index].getStateMode = mode;  
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

NTCAN_RESULT SMP::CMD_REFERENCE(int index){
	// Caution: need to know zero position of each module 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	int32_t module_id; 
	retValue = CMD_ACK(index);//CMD ACK first 

	cmsg.len=2;
	module_id = module[index].getId(); 
	cmsg.id=module_id|MSG_MASTER_TO_SLAVE;
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
	Sleep(5000);  // replaced with WaitForSingleObject(handle, time)
	return retValue;
}

NTCAN_RESULT SMP::CMD_STOP(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	int32_t module_id; 
	
	cmsg.len=2;
	module_id = module[index].getId(); 
	cmsg.id=module_id|MSG_MASTER_TO_SLAVE;
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

NTCAN_RESULT SMP::CMD_EMERGENCY_STOP(int index){

	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	int32_t module_id; 
	
	cmsg.len=2;
	module_id = module[index].getId(); 
	cmsg.id=module_id|MSG_MASTER_TO_SLAVE;

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
NTCAN_RESULT SMP::CMD_MOV_POS(int index){
	NTCAN_RESULT retValue; 
	// CMSG *cmsg; 
	//cmsg = new CMSG; 
	CMSG cmsg; 
	int32_t len = 1; 
	int32_t module_id; 
	for (int i=0; i<9; i++)
		cmsg.data[i] = 0;
	module_id = module[index].getId();
	cmsg.id = module_id|MSG_MASTER_TO_SLAVE;
	cmsg.len = 2; 
	cmsg.data[0] = 0x01; 
	cmsg.data[1] = _MOV_POS; 

	retValue = canSend(handle, &cmsg, &len);
	if (retValue != NTCAN_SUCCESS){
		cout <<"Failed to send CMD_MOV_POS! Error "<<retValue<<endl; 
		return retValue; 
	}
	return retValue;
}


NTCAN_RESULT SMP::CMD_MOV_POS(int index, float pos){
	return  0; 
}
NTCAN_RESULT SMP::CMD_MOV_POS(int index, float pos ,float vel, float acc, float cur){
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
	uint8_t *param_pos=(uint8_t*)new uint8_t[4];
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_acc=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];
	int32_t module_id; 

	float2Bit(pos, param_pos);
	float2Bit(vel, param_vel);
	float2Bit(acc, param_acc);
	float2Bit(cur, param_cur);

	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
		cmsg[2].data[i] = 0;
	}
	module_id = module[index].getId(); 
	cmsg[0].id = module_id|MSG_MASTER_TO_SLAVE;
	cmsg[1].id = module_id|MSG_MASTER_TO_SLAVE;
	cmsg[2].id = module_id|MSG_MASTER_TO_SLAVE;

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
	/*
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
	*/
	
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

NTCAN_RESULT SMP::CMD_MOV_VEL(int index, float vel, float cur){
	// this->checkStatus(index);
	// vel = this->checkVel(index,vel);
	// cur = this->checkCur(index,cur);

	NTCAN_RESULT retValue;
	CMSG* cmsg;
	cmsg=new CMSG[2];
	int32_t len=2;
	uint8_t *param_vel=(uint8_t*)new uint8_t[4];
	uint8_t *param_cur=(uint8_t*)new uint8_t[4];
	int32_t module_id; 

	float2Bit(vel, param_vel);
	float2Bit(cur, param_cur);
	
	for (int i=0; i<8; i++){
		cmsg[0].data[i] = 0;
		cmsg[1].data[i] = 0;
	}
	
	module_id = module[index].getId(); 
	cmsg[0].id = module_id|MSG_MASTER_TO_SLAVE;
	cmsg[1].id = module_id|MSG_MASTER_TO_SLAVE;
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

NTCAN_RESULT SMP::SET_TARGET_POS(int index, float val){
	// val = this->checkPos(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg; 
	int32_t len = 1; 
	uint8_t *param_val=(uint8_t*)new uint8_t[4]; 
	int32_t module_id; 
	float2Bit(val, param_val); 

	cmsg.len=6; 
	module_id = module[index].getId(); 
	cmsg.id=module_id|MSG_MASTER_TO_SLAVE; 

	for(int i=0; i<8; i++)
		cmsg.data[i] = 0; 
	cmsg.data[0] = 0x05; 
	cmsg.data[1] = _SET_TARGET_POS; 
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
NTCAN_RESULT SMP::SET_TARGET_POS_REL(int index, float val){
	// val = this->checkRelPos(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg; 
	int32_t len = 1; 
	uint8_t *param_val=(uint8_t*)new uint8_t[4]; 
	int32_t module_id; 
	float2Bit(val, param_val); 

	cmsg.len=6; 
	module_id = module[index].getId();
	cmsg.id=module_id|MSG_MASTER_TO_SLAVE; 

	for(int i=0; i<8; i++)
		cmsg.data[i] = 0; 
	cmsg.data[0] = 0x05; 
	cmsg.data[1] = _SET_TARGET_POS_REL; 
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

NTCAN_RESULT SMP::SET_TARGET_VEL(int index, float val){
	// val = this->checkVel(index, val); 
	NTCAN_RESULT retValue;
	CMSG cmsg;
	int32_t len=1;
	uint8_t *param_val=(uint8_t*)new uint8_t[4];
	int32_t module_id; 
	float2Bit(val, param_val);

	cmsg.len=6;
	module_id = module[index].getId(); 
	cmsg.id=module_id|MSG_MASTER_TO_SLAVE;

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

NTCAN_RESULT SMP::CMD_ACK(int index){
	NTCAN_RESULT retValue; 
	CMSG cmsg; 
	int32_t len = 1; 
	int32_t module_id; 

	cmsg.len = 2; 
	module_id = module[index].getId(); 
	cmsg.id = module_id|MSG_MASTER_TO_SLAVE; 
	for(int i = 0; i<8; i++)
		cmsg.data[i] = 0; 
	cmsg.data[0] = 0x01; 
	cmsg.data[1] = _CMD_ACK; 
	
	retValue = canSend(handle, &cmsg, &len); // acknowledgement
	if (retValue != NTCAN_SUCCESS ){
		cout << "Failed to send CMD_ACK(acknowledge pending error code)!" << retValue << endl;
		return NTCAN_TX_ERROR;
	}
	return retValue;
};
void SMP::float2Bit(float float_num, uint8_t* bit_num){
	union { 
		uint8_t n[4]; 
		float num;
	}tmp;
	tmp.num = float_num;
	for(int i=0; i<4; i++)
		bit_num[i]=tmp.n[i];
}
float SMP::bit2Float(uint8_t* bit_num){
	union { 
		uint8_t n[4]; 
		float num; 
	}tmp;
	for(int i=0; i<4; i++ )
		tmp.n[i] = bit_num[i];
	return tmp.num;
}



void SMP::threadOpen(){
	this->enableCanPoll(); 
	this->enableCanComm(); 
	this->enableEventHandle();

	this->pThreadPolling = (HANDLE)_beginthreadex(NULL, 0, SMP::canPollingThreadStartUp, this, CREATE_SUSPENDED, NULL); 
	ResumeThread(pThreadPolling); 
	this->pThreadHandling = (HANDLE)_beginthreadex(NULL, 0, SMP::eventThreadStartUp, this, CREATE_SUSPENDED, NULL); 
	ResumeThread(pThreadHandling); 
}
void SMP::threadClose()
{
	this->disableCanComm();    // communication flag
	this->disableCanPoll();		// thread polling
	this->disableEventHandle();	 // threa handling 	
	TerminateThread(pThreadPolling, 0); 
	TerminateThread(pThreadHandling, 0); 
	//ExitProcess(0); 
}

void SMP::enableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = true;			
	LeaveCriticalSection(&cSec);
}
void SMP::disableCanPoll(){
	EnterCriticalSection(&cSec);
	canPollActive = false;
	LeaveCriticalSection(&cSec);
}
void SMP::enableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = true;
	LeaveCriticalSection(&cSec);
}
void SMP::disableCanComm(){
	EnterCriticalSection(&cSec);
	canCommActive = false;
	LeaveCriticalSection(&cSec);
}
void SMP::enableEventHandle(){
	EnterCriticalSection(&cSec);
	eventActive = true;
	LeaveCriticalSection(&cSec);
}
void SMP::disableEventHandle(){
	EnterCriticalSection(&cSec);
	eventActive = false;
	LeaveCriticalSection(&cSec);
}; 
void SMP::canON(){
	this->canConnect(baud); 
	this->canModuleIdAdd(); 
};
void SMP::canOFF(){
	this->canModuleIdDelete();
	this->canDisconnect(); 	
}
NTCAN_HANDLE SMP::canConnect(uint32_t baud){
	NTCAN_RESULT retValue; 
	int net = 0; 
	uint32_t mode = 0; 
	int32_t txQueueSize = 40; 
	int32_t rxQueueSize = 40; 
	int32_t txTimeout = 100; 
	int32_t rxTimeout = 1000; 

	retValue = canOpen(net, mode, txQueueSize, rxQueueSize, txTimeout, rxTimeout, &handle);  // handle of CAN
	if (retValue != NTCAN_SUCCESS){
		cout<<"canOpen() failed with error "<<retValue<<" !"<<endl;
		return(NULL);
	}
	else 
		cout<<"Function canOpen() returned OK."<<endl;
	
	retValue = canSetBaudrate(handle, baud);    // baudrate of CAN
	if (retValue != NTCAN_SUCCESS){
		cout<<"canSetBaudrate() failed with error "<<retValue<<" !"<<endl;
		canClose(handle);
		return(NULL);
	}
	else 
		cout<<"Function canSetBaudrate() returned OK."<<endl;

	return handle;  //
}; 

NTCAN_RESULT SMP::canDisconnect(){
	NTCAN_RESULT retValue; 
	
	retValue = canClose(handle); 
	if(retValue != NTCAN_SUCCESS){
		cout<<"NTCAN net not closed! "<<endl; 
		return NTCAN_WRONG_DEVICE_STATE; 
	}
	else{
		cout<<"CAN-bus is closed. "<<endl; 
		return NTCAN_SUCCESS; 
	}
}

NTCAN_RESULT SMP::canModuleIdAdd(){
	NTCAN_RESULT	retValue; 
	int32_t id = 0; 
 	
	for (int i = 0; i<MODULE_NUM; i++){
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
		
		id = module[i].getId(); 
		id = id|MSG_SLAVE_TO_MASTER;
		retValue = canIdAdd(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdAdd() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdAdd() added Module ["<< i << "]|S->M to CAN. "<<endl;  
		}
		
		id = module[i].getId(); 
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

NTCAN_RESULT SMP::canModuleIdDelete(){
	NTCAN_RESULT	retValue; 
	int32_t id = 0; 
	for (int i = 0; i<MODULE_NUM; i++){
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
		
		id = module[i].getId();
		id = id|MSG_SLAVE_TO_MASTER;
		retValue = canIdDelete(handle, id); 
		if(retValue != NTCAN_SUCCESS){
			cout<<"canIdDelete() failed with error "<<retValue<<"!"<<endl; 
			return(NTCAN_ID_NOT_ENABLED); 
		}
		else{
			cout<< "Function canIdDelete() deleted Module ["<< i << "]|S->M in CAN. "<<endl;  
		}
		
		id = module[i].getId();
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

void SMP::errorHandler(int index){
	uint8_t module_error; 
	int32_t module_id; 
	module_id = module[index].getId(); 
	module_error = module[index].getErrorCode();  
	cout<<"Module "<< module_id << " encounters Error "<< module_error <<endl; 
	switch(module_error)
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
		break;
	case	INFO_FAILED:
		cout<<"Error code: 0x05. Command failed!"<<endl;
		CMD_STOP(index);
		break;
	case	INFO_NOT_REFERENCED:     // not referenced
		cout<<"Error code: 0x06(INFO). Not referenced!"<<endl;
		CMD_STOP(index);
		// CMD_REFERENCE(index); 
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
		//canInitial(); 
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
		break;
	case	INFO_MESSAGE_LENGTH:
		cout<<"Error code: 0x1D. D-Len does not match."<<endl;
		CMD_STOP(index);
		break;
	case	INFO_WRONG_PARAMETER:
		cout<<"Error code: 0x1E. Wrong parameter!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_WRONG_RAMP_TYPE:
		cout<<"Error code: 0xC8. No valid motion profile!"<<endl;;
		CMD_STOP(index); 
		// canDisconnect();
		// exit(0);
		break;
	case	ERROR_CONFIG_MEMORY:
		cout<<"Error code: 0xD2. Configuration range is incorrect!"<<endl;
		CMD_STOP(index); 
		// canDisconnect();
		// exit(0);
		break;
	case	ERROR_SOFT_LOW:
		cout<<"Error code: 0xD5. Module exceeded software low limit!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_SOFT_HIGH:
		cout<<"Error code: 0xD6. Module exceeded software high limit!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_EMERGENCY_STOP:
		cout<<"Error code: 0xD9. Emergency Stop!"<<endl;
		// CMD_REFERENCE(index); 
		break;
	case	ERROR_TOW:           // A common err.
		cout<<"Error code: 0xDA. Towing Error! Reducing load of module!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_TOO_FAST:
		cout<<"Error code: 0xE4. Too Fast. Reducing current of module!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_COMMUTATION: 
		cout<<"Error code: 0xDD. Module fails to commutate."<<endl; 
		CMD_STOP(index);
		break; 
	case	ERROR_FRAGMENTATION:
		cout<<"Error code: 0xDC. Error in fragmentation. Data packets lost!"<<endl;
		break;
	case	ERROR_CURRENT:
		cout<<"Error code: 0xDE. Current too large! Reducing load of module!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_I2T:
		cout<<"Error code: 0xDF. I2T Error!  Reducing load of module!"<<endl;
		CMD_STOP(index);
		break;
	case	ERROR_INITIALIZE:
		cout<<"Error code: 0xE0. Module could not be initialized"<<endl;
		//canDisconnect();
		break;
	case	ERROR_TEMP_LOW:
		cout<<"Error code: 0x70. Temprature of module is too low!"<<endl;
		CMD_STOP(index);
		//canDisconnect();
		exit(0);
		break;
	case	ERROR_TEMP_HIGH:
		cout<<"Error code: 0x71. Temprature of module is too high!"<<endl;
		CMD_STOP(index);
		//canDisconnect();
		exit(0);
		break;
	case	ERROR_LOGIC_LOW:
		cout<<"Error code: 0x72. Logic voltage is too low!"<<endl;
		//canDisconnect();
		exit(0);
		break;
	case	ERROR_LOGIC_HIGH:
		cout<<"Error code: 0x73. Logic voltage is too high!"<<endl;		
		//canDisconnect();
		exit(0);
		break;
	case	ERROR_MOTOR_V_LOW:
		cout<<"Error code: 0x74. Motor voltage is too low!"<<endl;
		//canDisconnect();
		exit(0);
		break;
	case	ERROR_MOTOR_V_HIGH:
		cout<<"Error code: 0x75. Motor voltage is too high!"<<endl;
		//canDisconnect();
		exit(0);
		break;
	case	ERROR_CABLE_BREAK:
		cout<<"Error code: 0x76. Cable is defective!"<<endl;
		CMD_STOP(index);
		// canDisconnect();
		exit(0);
		break;
	default: 
		cout<<"Error not defined yet"<<endl; 
		CMD_STOP(index); 
	}
	CMD_ACK(index);
}