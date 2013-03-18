#include <iostream>
#include "module.h"

using std::cout; 
using std::endl;

/****************
Data struct Module
1. attributes (id, statusCode, errorCode, mode)
2. func to set, get and update 
******************/

Module::Module(){
    id_ = 0;
    mode_ = 0x00;
    status_code = 0x00;
    error_code = 0x00;
    position_ = 0;
    velocity_ = 0;
    current_ = 0;
    acceleration_ = 0;
}

Module::~Module(){
}

void Module::set_id(int32_t id){
	id_ = id; 
}

void Module::set_status_code(uint8_t status_code){
	status_code_ = status_code; 
}

void Module::set_error_code(uint8_t error_code){
	error_code_ = error_code; 
}

void Module::set_mode(uint8_t mode){
	mode_ = mode; 
}

void Module::set_position(float position)
{
	position_ = position; 
}

void Module::set_velocity(float velocity)
{
	velocity_ = velocity; 
}

void Module::set_acceleration(float acceleration)
{
	acceleration_ = acceleration; 
}

void Module::set_current(float current)
{
	current_ = current; 
}

int32_t	Module::id(){
	return id_; 
}

uint8_t Module::status_code(){
	return status_code_; 
}

uint8_t	Module::error_code(){
	return error_code; 
}

uint8_t Module::mode(){
	return mode_; 
}

float Module::position(){
	return position_; 
}

float Module::velocity(){
	return velocity_; 
}

float Module::acceleration(){
	return acceleration_; 
}

float Module::current(){
	return current_; 
}

bool Module::CheckReferenced(){
	return (status_code_ & kReferencedTrue == 1) ? true : false; 
} 

bool Module::CheckMoving(){
    return (status_code_ & kMovingTrue ==1) ? true : false;
} 

bool Module::CheckError(){
    return (status_code_ & kErrorTrue == 1) ? true : false;
} 

bool Module::CheckBrake(){
    return (status_code_ & kBrakeTrue == 1) ? true : false;
} 

bool Module::CheckMoveEnd(){
    return (status_code_ & kMoveEndTrue == 1) ? true : false;
} 

bool Module::CheckPosReached(){
    return (status_code_ & kPositionReachedTrue == 1) ? true : false;
} 
