#include "ntcan.h"
#include <windows.h>
#include <process.h>
#include <iostream>
#include <vector>
using namespace std; 

// some constants 
#define ID_START					8
#define ID_END						15
#define SUCCESS						1
#define FAILURE						0
#define OUT_RANGE					-10
#define IN_RANGE					10
#define OFFSET						8 

//Group ID
#define MSG_MASTER					0x0500
#define MSG_SLAVE					0x0700
#define MSG_ERROR					0x0300
////////////////
#define GET_ID 0x00FF
#define EMPTY_Q 32
#define UNKNOWN_MSG 90
#define OUT_OF_Q 91
#define RX_TIME_OUT 92
#define PARSE_SUCCESS 93
#define FRAGGING 94
#define	REFERENCE_FAIL 95
#define	CMD_STOP_FAIL 96
#define	ACK_FAIL 96
#define EVENT_REACHED 97
#define EVENT_NULL 98

#define CHECK_REFERENCED 0x01
#define CHECK_MOVING 0x02
#define CHECK_ERROR 0x10
#define CHECK_BRAKE 0x20
#define CHECK_MOV_END 0x40
#define CHECK_POS_REACHED 0x80
/////////////////

#define _SET_TARGET_VEL				0xA0
#define _SET_TARGET_ACC				0xA1
#define _SET_TARGET_JER				0xA2
#define _SET_TARGET_CUR				0xA3
#define _SET_TARGET_TIME			0xA4

//Command: Motion Codes
#define _CMD_STOP					0x91
#define _CMD_EMERGENCY_STOP			0x90
#define _CMD_REFERENCE				0x92
#define _CMD_REFERENCE_HAND			0x97

#define	_MOV_POS					0xB0
#define	_MOV_POS_REL				0xB8
#define _MOV_POS_TIME				0xB1
#define _MOV_POS_TIME_REL			0xB9

#define _MOV_CUR					0xB3
#define	_MOV_VEL					0xB5
#define	_MOV_GRIP					0xB7

#define _MOV_POS_LOOP				0xBA
#define _MOV_POS_TIME_LOOP			0xBB
#define _MOV_POS_REL_LOOP			0xBC
#define _MOV_POS_TIME_REL_LOOP		0xBD

//Commend: Impulse Message & others
#define _GET_STATE					0x95
#define _FRAG_BEGIN					0x84
#define _FRAG_MIDDLE				0x85
#define _FRAG_END					0x86

#define _MOV_END				0x93
#define _POS_REACHED			0x94

#define _SET_CONFIG					0x81
#define _GET_CONFIG					0x80

//Error Messages
#define _CMD_ERROR						0x88
#define _CMD_WARNING					0x89
#define _CMD_INFO						0x8A
#define _CMD_ACK						0x8B

//Error Codes List
#define INFO_BOOT				0x01    // 0x0001
#define	INFO_NO_FREE_SPACE		0x02
#define	INFO_NO_RIGHTS			0x03     //
#define	INFO_UNKNOWN_COMMAND	0x04     //
#define INFO_FAILED				0x05     //
#define	INFO_NOT_REFERENCED		0x06     // 
#define INFO_SEARCH_SINE_VECTOR	0x07    // 0x0007
#define	INFO_NO_ERRORS			0x08    // 0x0008
#define INFO_COMMUNICATE_ERROR  0x09     //
#define INFO_TIMEOUT			0x10    //
#define INFO_WRONG_BAUDRATE		0x16     //
#define	INFO_CHECKSUM			0x19     //
#define INFO_MESSAGE_LENGTH		0x1D     //
#define INFO_WRONG_PARAMETER	0x1E     //
#define INFO_PROGRAM_END		0x1F     
#define INFO_TRIGGER			0x40               // 0x0040
#define INFO_READY				0x41				//0x0041	 
#define INFO_GUI_CONNECTED		0x42                //0x0042    
#define INFO_GUI_DISCONNECTED	0x43                //0x0043     
#define INFO_PROGRAM			0x44
#define ERROR_WRONG_RAMP_TYPE	0xC8    //
#define ERROR_CONFIG_MEMORY		0xD2	//
#define ERROR_PROGRAM_MEMORY	0xD3    //
#define ERROR_INVALID_PHRASE    0xD4    //   
#define ERROR_SOFT_LOW			0xD5    //
#define ERROR_SOFT_HIGH			0xD6	//
#define ERROR_PRESSURE			0xD7
#define ERROR_SERVICE			0xD8    //
#define ERROR_EMERGENCY_STOP	0xD9    //
#define ERROR_TOW				0xDA    //  common 
#define ERROR_TOO_FAST			0xE4    // 
#define ERROR_VPC3				0xDB   //
#define ERROR_FRAGMENTATION		0xDC   //
#define ERROR_COMMUTATION		0xDD   //   
#define ERROR_CURRENT			0xDE    //
#define	ERROR_I2T				0xDF    //
#define ERROR_INITIALIZE		0xE0    //
#define ERROR_INTERNAL			0xE1   //
#define ERROR_HARD_LOW			0xE2
#define ERROR_HARD_HIGH			0xE3
#define ERROR_TEMP_LOW			0x70   //
#define ERROR_TEMP_HIGH			0x71   //
#define ERROR_LOGIC_LOW			0x72   //
#define ERROR_LOGIC_HIGH		0x73   //
#define ERROR_MOTOR_V_LOW		0x74   //
#define ERROR_MOTOR_V_HIGH		0x75    //
#define ERROR_CABLE_BREAK		0x76    //    common 
#define ERROR_MOTOR_TEMP		0x78
////new included
#define ERROR_MATH				0xEC
#define ERROR_OVERSHOOT			0x82
#define ERROR_RESOLVER_CHECK_FAILED	0xEB 
#define INFO_UNKNOWN_AXIS_INDEX	0x11
#define ERROR_HARDWARE_VERSION	0x83 

//A State Description  

// Range & Customed Values
const float MinPos[8]			={-40,	-90,	-90,	0,		-90,	-90,	-90,	0};
const float MaxPos[8]			={20,	 45,	90,		90,		90,		90,		90,		45};
const float OrigPos[8]			={}; 

const float MaxVel[8]			={10,	20,		20,		10,		20,		20,		20,		10};
const float CustomedVel[8]		={5,	 10,	10,		5,		10,		10,		10,		5};

const float MaxAcc[8]			={20,	20,		20,		20,		20,		20,		20,		10};
const float CustomedAcc[8]		={8,	8,		8,		8,		8,		8,		8,		5};

const float MaxCur[8]			={25,	25,		30,		12,		12,		8,		8,		6};  //
const float CustomedCur[8]		={10,	15,		12,		6,		6,		4,		4,		2};  //

//const int	GearRatio[8]		={161,	161,	161,	161,	161,	121,	101,	1};
//const float MaxJerk[8]			={};
//const float MyMaxJerk[8]		={};


typedef struct
{
	int32_t	id;				//range: 8~15
	uint8_t	status;			// word of state 8-bit (referenced - 01; error - 05;)
	uint8_t	errorCode;		// word of error type
	uint8_t	getStateMode;

	float	position;
	float	velocity;
	float	current;
	
	bool	referenced;
	bool	moving;
	bool	brake;
	bool	moveEnd;
	bool	posReached;
	bool	error; 
	// bool	hold;
	bool	movBack;   // movingBack
	bool	movForth;  // movingForth

	int		mEvent;   // EVENT_REACHED(97); EVENT_NULL(98)

} MODULE;

class ARM
{
public:
	ARM();
	~ARM(); 
	// connect & disconnect
	NTCAN_RESULT	canInitial();
	NTCAN_HANDLE	canConnect(uint32_t baud);	// open CAN communication
	NTCAN_RESULT	canScanBus(NTCAN_HANDLE handle);	// scan bus
	NTCAN_RESULT	canDisconnect();	// close CAN communication

	// Error 
	NTCAN_RESULT	CMD_ACK(int index);
	
	// Start & Stop 
	// NTCAN_RESULT	CMD_MOV_END()
	// NTCAN_RESULT	CDM_POS_REACHED()
	NTCAN_RESULT	CMD_GET_STATE(int index,float time, uint8_t mode);
	NTCAN_RESULT	CMD_REFERENCE(int index);
	NTCAN_RESULT	CMD_STOP(int index);
	NTCAN_RESULT	CMD_EMERGENCY_STOP(int index);
	
	// Motions 
	NTCAN_RESULT	CMD_MOV_POS(int index, float pos);          // 0xB0
	NTCAN_RESULT	CMD_MOV_POS(int index, float pos, float vel, float acc, float cur); 
	NTCAN_RESULT	CMD_MOV_POS_REL(int index, float pos);      //0xB8
	NTCAN_RESULT	CMD_MOV_POS_REL(int index, float pos, float vel, float acc, float cur);
	NTCAN_RESULT	CMD_MOV_POS_TIME(int index, float pos);     //0xB1
	NTCAN_RESULT	CMD_MOV_POS_TIME(int index, float pos, float vel, float acc, float cur, float time);
	NTCAN_RESULT	CMD_MOV_POS_TIME_REL(int index, float pos);  //0xB9
	NTCAN_RESULT	CMD_MOV_POS_TIME_REL(int index, float pos, float vel, float acc, float cur, float time);
		
	NTCAN_RESULT	CMD_MOV_POS_LOOP(int index, float pos);      //0xBA
	NTCAN_RESULT	CMD_MOV_POS_LOOP(int index, float pos, float vel, float acc, float cur);
	NTCAN_RESULT	CMD_MOV_POS_REL_LOOP(int index, float pos);   //0xBC
	NTCAN_RESULT	CMD_MOV_POS_REL_LOOP(int index, float pos, float vel, float acc, float cur);
	NTCAN_RESULT	CMD_MOV_POS_TIME_LOOP(int index, float pos);      //0xBB
	NTCAN_RESULT	CMD_MOV_POS_TIME_LOOP(int index, float pos, float vel, float acc, float cur, float time);
	NTCAN_RESULT	CMD_MOV_POS_TIME_REL_LOOP(int index, float pos);      //0xBD
	NTCAN_RESULT	CMD_MOV_POS_TIME_REL_LOOP(int index, float pos, float vel, float acc, float cur, float time);

		// Motions in new version
	NTCAN_RESULT	CMD_MOV_POS(int index);				//0xB0
	NTCAN_RESULT	CMD_MOV_POS_REL(int index);			//0xB8
	NTCAN_RESULT	CMD_MOV_POS_TIME(int index);		//0xB1
	NTCAN_RESULT	CMD_MOV_POS_TIME_REL(int index);	//0xB9

	NTCAN_RESULT	CMD_MOV_CUR(int index, float cur);      //0xB3
	NTCAN_RESULT	CMD_MOV_VEL(int index, float vel);      //0xB5
	NTCAN_RESULT	CMD_MOV_VEL(int index, float vel, float cur);
	NTCAN_RESULT	CMD_MOV_GRIP(int index, float cur);     //0xB7

	NTCAN_RESULT	SET_TARGET_VEL(int index, float val);   //0xA0
	NTCAN_RESULT	SET_TARGET_ACC(int index, float val);	 //0xA1
	NTCAN_RESULT	SET_TARGET_JER(int index, float val);    //0xA2
	NTCAN_RESULT	SET_TARGET_CUR(int index, float val);   //0xA3
	NTCAN_RESULT	SET_TARGET_TIME(int index, float val);  //0xA4
	NTCAN_RESULT	SET_TARGET_POS(int index, float val);   // 0xA6
	NTCAN_RESULT	SET_TARGET_POS_REL(int index, float val); //0xA7


	void			checkStatus(int index);	//
	int				checkPos(int index, float pos); 
	int				checkRelPos(int index, float relpos);
	float			checkVel(int index, float vel); 
	float			checkAcc(int index, float acc); 
	float			checkCur(int index, float cur); 
	float			checkJer(int index, float jer); 
	float			checkTime(int index, float time, float posTar); 
	float			checkTimeRel(int index, float time, float posTar); 

	////////////////////////
	void			float2Bit(float float_num, uint8_t* bit_num);
	float			bit2Float(uint8_t* bit_num);
	int				canMsgPoll();
	int				procNextMsg(int count, CMSG* cmsg); // parse next message in the buffer
	int				parseFragMsg(int mIndex);
	void			canPolling();
	void			eventHandling();
	void			errorHandler(int index);
	void			callBackReached(int index);
	void			callBackReachedNotBrake(int index); 


	void			enableCanPoll();
	void			disableCanPoll();
	void			enableCanComm();
	void			disableCanComm();
	void			enableEvent();
	void			disableEvent();
	void			enableHold(int index);
	void			disableHold(int index);

	CRITICAL_SECTION cSec; 

	// for openThread functions 
	void			openThread(); 
	void			closeThread(); 
	static unsigned __stdcall canPollingThreadStartUp(void * pThis){
		ARM* pARM = (ARM*)pThis;
		pARM->canPolling();

		return 1;
	}
	static unsigned __stdcall eventThreadStartUp(void * pThis){
		ARM* pARM = (ARM*)pThis;
		pARM->eventHandling();

		return 1;
	}
	HANDLE		pThreadPolling;			
	HANDLE		pThreadHandling;			

	MODULE			module[8];

private:

	NTCAN_HANDLE		handle;  
	uint32_t			baud;
	// int32_t				idStart;
	// int32_t				idEnd;

	//buffers 
	uint8_t				*pos_buffer;
	uint8_t				*vel_buffer;
	uint8_t				*cur_buffer;
	// CMSG				fragStartBuf;
	// CMSG				fragMidBuf;
	CMSG				pollBuf[40]; // MSG Read from module; 
	CMSG				fragBuf[8][40]; // parse frag. MSG
	int					fragLen[8];

	float				v0;
	float				v3;
	float				v5;
	float				v6; 
	float				v7;
	
	bool				canCommActive;
	bool				canPollActive;
	bool				eventActive;

};

