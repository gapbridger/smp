#include "SMP.h"
#include <iostream>
using namespace std;

//some constants 
#define SUCCESS						1
#define FAILURE						0
#define OUT_RANGE					-10
#define IN_RANGE					10
#define OFFSET						8 

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

// // Range & Customed Values
const float MinPos[8]			={-40,	-90,	-90,	0,		-90,	-90,	-90,	0};
const float MaxPos[8]			={20,	 45,	90,		90,		90,		90,		90,		45};
const float OrigPos[8]			={}; 

const float MaxVel[8]			={10,	20,		20,		10,		20,		20,		20,		10};
const float CustomedVel[8]		={5,	 10,	10,		5,		10,		10,		10,		5};

const float MaxAcc[8]			={20,	20,		20,		20,		20,		20,		20,		10};
const float CustomedAcc[8]		={8,	8,		8,		8,		8,		8,		8,		5};

const float MaxCur[8]			={25,	25,		30,		12,		12,		8,		8,		6};  //
const float CustomedCurLimit[8]		={10,	15,		12,		6,		6,		4,		4,		1.8};  //


class ARM
{
public:
	ARM(); 
	~ARM(); 
	MODULE			*moduleArm;
	int				moduleNum; 
	int32_t			idStart; 
	int32_t			idEnd; 
	// NTCAN_RESULT	moduleIdAdd(NTCAN_HANDLE handle); 
	// NTCAN_RESULT	moduleIdDelete(); 
	NTCAN_RESULT	CALL_CMD_MOV_POS(int index, SMP smp, float pos, float vel, float acc, float cur); 
	NTCAN_RESULT	CALL_CMD_STATE_UPDATE(int index, SMP smp, float timeFreq, uint8_t mode); 
	NTCAN_RESULT	CALL_CMD_ACK(int index, SMP smp); 
	NTCAN_RESULT	CALL_CMD_STOP(int index, SMP smp); 

protected:
	void			checkStatus(int index, SMP smp);	//
	int				checkPos(int index, float pos); 
	int				checkRelPos(int index, float relPos);
	float			checkVel(int index, float vel); 
	float			checkAcc(int index, float acc); 
	float			checkCur(int index, float cur); 
	// float			checkJer(int index, float jer); 
	// float			checkTime(int index, float time, float posTar); 
	// float			checkTimeRel(int index, float time, float posTar); 
	CRITICAL_SECTION	cSec; 
}; 
