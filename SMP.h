#include "ntcan.h"
#include <windows.h>
#include <process.h>
#include <iostream>
#include <vector>
using namespace std; 

/*******************
CMD Code List
********************/
#define _CMD_STOP					0x91
#define _CMD_EMERGENCY_STOP			0x90
#define _CMD_REFERENCE				0x92
#define _CMD_REFERENCE_HAND			0x97
#define _MOV_END					0x93
#define _POS_REACHED				0x94
#define _GET_STATE					0x95

#define _SET_CONFIG					0x81
#define _GET_CONFIG					0x80
#define _CMD_ERROR					0x88
#define _CMD_WARNING				0x89
#define _CMD_INFO					0x8A
#define _CMD_ACK					0x8B
#define _GET_DETAILED_ERROR_INFO	0x96

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

#define _SET_TARGET_VEL				0xA0
#define _SET_TARGET_ACC				0xA1
#define _SET_TARGET_JER				0xA2
#define _SET_TARGET_CUR				0xA3
#define _SET_TARGET_TIME			0xA4
#define _SET_TARGET_POS				0xA6	// cmd in new version  
#define _SET_TARGET_POS_REL			0xA7	// cmd in new version

#define _FRAG_BEGIN					0x84
#define _FRAG_MIDDLE				0x85
#define _FRAG_END					0x86

/*********************
Error Codes List
**********************/
#define INFO_BOOT					0x01		// 0x0001
#define	INFO_NO_RIGHTS				0x03		
#define	INFO_UNKNOWN_COMMAND		0x04		
#define INFO_FAILED					0x05		
#define	INFO_NOT_REFERENCED			0x06		 
#define INFO_SEARCH_SINE_VECTOR		0x07		// 0x0007
#define	INFO_NO_ERRORS				0x08		// 0x0008
#define INFO_COMMUNICATE_ERROR		0x09		
#define INFO_TIMEOUT				0x10		
#define INFO_WRONG_BAUDRATE			0x16		
#define	INFO_CHECKSUM				0x19		
#define INFO_MESSAGE_LENGTH			0x1D		
#define INFO_WRONG_PARAMETER		0x1E		
#define ERROR_WRONG_RAMP_TYPE		0xC8    
#define ERROR_CONFIG_MEMORY			0xD2	
#define ERROR_PROGRAM_MEMORY		0xD3    
#define ERROR_INVALID_PHRASE		0xD4      
#define ERROR_SOFT_LOW				0xD5    
#define ERROR_SOFT_HIGH				0xD6	
#define ERROR_PRESSURE				0xD7
#define ERROR_SERVICE				0xD8    
#define ERROR_EMERGENCY_STOP		0xD9    
#define ERROR_TOW					0xDA		//  common 
#define ERROR_TOO_FAST				0xE4    
#define ERROR_VPC3					0xDB  
#define ERROR_FRAGMENTATION			0xDC   
#define ERROR_COMMUTATION			0xDD   
#define ERROR_CURRENT				0xDE    
#define	ERROR_I2T					0xDF    
#define ERROR_INITIALIZE			0xE0    
#define ERROR_INTERNAL				0xE1   
#define ERROR_TEMP_LOW				0x70   
#define ERROR_TEMP_HIGH				0x71   
#define ERROR_LOGIC_LOW				0x72   
#define ERROR_LOGIC_HIGH			0x73   
#define ERROR_MOTOR_V_LOW			0x74   
#define ERROR_MOTOR_V_HIGH			0x75    
#define ERROR_CABLE_BREAK			0x76		// common 
//old included 
#define	INFO_NO_FREE_SPACE			0x02
#define INFO_PROGRAM_END			0x1F     
#define INFO_TRIGGER				0x40				//0x0040
#define INFO_READY					0x41				//0x0041	 
#define INFO_GUI_CONNECTED			0x42                //0x0042    
#define INFO_GUI_DISCONNECTED		0x43                //0x0043     
#define INFO_PROGRAM				0x44
#define ERROR_MOTOR_TEMP			0x78
#define ERROR_HARD_LOW				0xE2
#define ERROR_HARD_HIGH				0xE3
//new included
#define ERROR_MATH					0xEC
#define ERROR_OVERSHOOT				0x82
#define ERROR_RESOLVER_CHECK_FAILED	0xEB 
#define INFO_UNKNOWN_AXIS_INDEX		0x11
#define ERROR_HARDWARE_VERSION		0x83 

/************************
Const
*************************/
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


typedef struct
{
	int32_t	id;				
	uint8_t	status;			// Byte of state 8-bit 
	uint8_t	errorCode;		// Byte of error type
	uint8_t	getStateMode;
	
	bool	referenced;     // status: 0x01 
	bool	moving;			// status: 0x02
	bool	error;			// status: 0x10
	bool	brake;			// status: 0x20
	bool	moveEnd;		// status: 0x40
	bool	posReached;		// status: 0x80
	bool	progMode;		// status: 0x04
	bool	warning;		// status: 0x08

		
	float	position;		
	float	velocity;		
	float	current;
	int		mEvent;			// EVENT_REACHED(97); EVENT_NULL(98)

} MODULE;

class KERNEL
{
public:
	KERNEL();			// Module_Initializaation; Open_Thread;   
	~KERNEL();			// Close_Thread 
	void			threadOpen();
	void			threadClose(); 
	// MOVING CMD
	NTCAN_RESULT	CMD_GET_STATE(int index,float time, uint8_t mode);   // mode: 01-pos; 02-vel; 04-cur
	NTCAN_RESULT	CMD_REFERENCE(int index);
	NTCAN_RESULT	CMD_STOP(int index);
	NTCAN_RESULT	CMD_EMERGENCY_STOP(int index);
	NTCAN_RESULT	CMD_ACK(int index); 

	NTCAN_RESULT	CMD_MOV_POS(int index);
	NTCAN_RESULT	CMD_MOV_POS(int index, float pos); 
	NTCAN_RESULT	CMD_MOV_POS(int index, float pos, float vel, float acc, float cur);
	NTCAN_RESULT	SET_TARGET_POS(int index, float pos); 
	NTCAN_RESULT	SET_TARGET_POS_REL(int index, float posRel); 
	// NTCAN_RESULT	SET_TARGET_VEL(int index, float vel); 

	static unsigned __stdcall canPollingThreadStartUp(void * pThis){
		KERNEL* pKERNEL = (KERNEL*)pThis;
		pKERNEL->canPolling();
		return 1;
	}
	static unsigned __stdcall eventThreadStartUp(void * pThis){
		KERNEL* pKERNEL = (KERNEL*)pThis;
		pKERNEL->eventHandling();
		return 1;
	}
	HANDLE			pThreadPolling;			
	HANDLE			pThreadHandling;	
	MODULE			module[8]; 

protected: 
	NTCAN_HANDLE	handle; 
	uint32_t		baud;
	uint8_t				*pos_buffer;
	uint8_t				*pos_rel_buffer; 
	uint8_t				*vel_buffer;
	uint8_t				*cur_buffer;
	CMSG				pollBuf[40];		// MSG Read from module; 
	CMSG				fragBuf[8][40];	
	int					fragLen[8];
	CRITICAL_SECTION cSec; 

	NTCAN_RESULT	canPolling(); 
	void			eventHandling(); 
	void			errorHandler(); 

	void			enableCanPoll();
	void			disableCanPoll();
	void			enableCanComm();
	void			disableCanComm();
	bool			canCommActive;
	bool			canPollActive;
 
	// buffers & stacks  
	// parse frag. MSG


	void			float2Bit(float float_num, uint8_t* bit_num);
	float			bit2Float(uint8_t* bit_num);

private: 
	int				procNextMsg(int count, CMSG* cmsg); // parse next message in the buffer
	int				parseFragMsg(int mIndex);

	
}