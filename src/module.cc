#include <iostream>
#include "MODULE.h"
using namespace std; 

/****************
Data struct MODULE
1. attributes (id, statusCode, errorCode, mode)
2. func to set, get and update 
******************/


MODULE::MODULE(){
	uint8_t module_error = 0x00, module_status = 0x00, module_mode = 0x00; 
	int32_t module_id = 0; 
	this->setId(module_id);
	this->setStatusCode(module_status); 
	this->setErrorCode(module_error);
	this->setMode(module_mode); 
}
MODULE::~MODULE(){
	
}
void MODULE::setId(int32_t _id){
	this->id = _id; 
}
void MODULE::setStatusCode(uint8_t _statusCode){
	this->statusCode = _statusCode; 
}
void MODULE::setErrorCode(uint8_t _errorCode){
	this->errorCode = _errorCode; 
}
void MODULE::setMode(uint8_t _mode){
	this->mode = _mode; 
}
void MODULE::updatePos(float _pos)
{
	this->pos = _pos; 
}
void MODULE::updateVel(float _vel)
{
	this->vel = _vel; 
}
void MODULE::updateAcc(float _acc)
{
	this->acc = _acc; 
}
void MODULE::updateCur(float _cur)
{
	this->cur = _cur; 
}
int32_t	MODULE::getId(){
	return this->id; 
}
uint8_t MODULE::getStatusCode(){
	return this->statusCode; 
}
uint8_t	MODULE::getErrorCode(){
	return this->errorCode; 
}
uint8_t MODULE::getMode(){
	return this->mode; 
}
float	MODULE::getPos(){
	return this->pos; 
}
float	MODULE::getVel(){
	return this->vel; 
}
float	MODULE::getAcc(){
	return this->acc; 
}
float	MODULE::getCur(){
	return this->cur; 
}

bool MODULE::checkReferenced(){
	bool retValue; 		
	retValue =  (((this->statusCode) & (TRUE_REFERENCED ==1))? true : false );
	return retValue; 
} 
bool MODULE::checkMoving(){
	bool retValue; 	
	retValue =  (((this->statusCode) & (TRUE_MOVING ==1))? true : false );
	return retValue; 
} 
bool MODULE::checkError(){
	bool retValue; 		
	retValue =  (((this->statusCode) & (TRUE_ERROR ==1))? true : false );
	return retValue; 
} 
bool MODULE::checkBrake(){
	bool retValue; 		
	retValue =  (((this->statusCode) & (TRUE_BRAKE ==1))? true : false );
	return retValue; 
} 
bool MODULE::checkMoveEnd(){
	bool retValue; 	
	retValue =  (((this->statusCode) & (TRUE_MOV_END ==1))? true : false );
	return retValue; 
} 
bool MODULE::checkPosReached(){
	bool retValue; 		
	retValue =  (((this->statusCode) & (TRUE_POS_REACHED ==1))? true : false );
	return retValue; 
} 
