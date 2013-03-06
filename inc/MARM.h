// #include "SMP.h"
#include "ARM.h"  
#include <iostream>
using namespace std; 

/*
class ARM_G1
{
public:
	ARM_G1(); 
	~ARM_G1(); 
	MODULE			*moduleG1; 
	int				moduleNum; 
	int32_t			idStart; 
	int32_t			idEnd; 
	NTCAN_RESULT	CALL_CMD_MOV_POS(int index, SMP smp, float pos, float vel, float acc, float cur); 
	NTCAN_RESULT	CALL_CMD_STATE_UPDATE(int index, SMP smp, float timeFreq, uint8_t mode); 
	NTCAN_RESULT	CALL_CMD_ACK(int index, SMP smp); 
	NTCAN_RESULT	DOUBLE_MODULE2STEP_MOTION(SMP smp, int index_01, int index_02, float pos_01, float pos_02); 


private:
	void			checkStatus(int index, SMP smp);	//
	int				checkPos(int index, float pos); 
	int				checkRelPos(int index, float relPos);
	float			checkVel(int index, float vel); 
	float			checkAcc(int index, float acc); 
	float			checkCur(int index, float cur); 

	CRITICAL_SECTION	cSec; 

}; 

class ARM_G2
{
public:
	ARM_G2(); 
	~ARM_G2(); 
	MODULE			*moduleG2; 
	int				moduleNum; 
	int32_t			idStart; 
	int32_t			idEnd; 
	NTCAN_RESULT	CALL_CMD_MOV_POS(int index, SMP smp, float pos, float vel, float acc, float cur); 
	NTCAN_RESULT	CALL_CMD_STATE_UPDATE(int index, SMP smp, float timeFreq, uint8_t mode); 
	NTCAN_RESULT	CALL_CMD_ACK(int index, SMP smp); 

private:
	void			checkStatus(int index, SMP smp);	//
	int				checkPos(int index, float pos); 
	int				checkRelPos(int index, float relPos);
	float			checkVel(int index, float vel); 
	float			checkAcc(int index, float acc); 
	float			checkCur(int index, float cur); 

	CRITICAL_SECTION	cSec; 
}; 

*/

class ARM_G1: public ARM
{
public: 
	ARM_G1();
	~ARM_G1();
	NTCAN_RESULT DOUBLE_MODULE2STEP_MOTION(SMP smp, int index_01, int index_02, float pos_01, float pos_02); 
}; 

class ARM_G2: public ARM
{
public:
	ARM_G2(); 
	~ARM_G2(); 
	NTCAN_RESULT SINGLE_MODULE2STEP_MOTION(SMP smp, int index, float pos_01, float pos_02);
}; 
