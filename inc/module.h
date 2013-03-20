#include "ntcan.h"

// using one set
const uint8_t kReferencedTrue = 0x01;
const uint8_t kMovingTrue = 0x02;
const uint8_t kErrorTrue = 0x10;
const uint8_t kBrakeTrue = 0x20;
const uint8_t kMoveEndTrue = 0x40;
const uint8_t kPositionReachedTrue = 0x80;

const uint8_t kReferencedFalse = 0xFE;
const uint8_t kMovingFalse = 0xFD;
const uint8_t kErrorFalse = 0xEF;
const uint8_t kBrakeFalse = 0xDF;
const uint8_t kMoveEndFalse = 0xBF;
const uint8_t kPositionReachedFalse = 0x7F;

class Module{
public: 
    // constructors
	Module(); 
	~Module(); 
    // mutators
	void set_id(int32_t id); 
	void set_status_code(uint8_t status_code); 
	void set_error_code(uint8_t error_code); 
	void set_mode(uint8_t mode); 
	void set_position(float position); 
	void set_velocity(float velocity); 
	void set_acceleration(float acceleration); 
	void set_current(float current); 
	void set_time_required(float time);
    // accessors
	int32_t	id() const; 
	uint8_t status_code() const; 
	uint8_t	error_code() const; 
	uint8_t	mode() const;    // mode: para. in CMD_GET_STATE
	float position() const; 
	float velocity() const; 
	float acceleration() const; 
	float current() const; 
	float time_required() const;
    // check status
	bool CheckReferenced() const;
	bool CheckMoving() const; 
	bool CheckError() const; 
	bool CheckBrake() const; 
	bool CheckMoveEnd() const; 
	bool CheckPositionReached() const; 

private:
	int32_t	id_;				
    uint8_t	mode_;
	uint8_t	status_code_; // Byte of status 8-bit 
	uint8_t	error_code_; // Byte of error type
	float position_;		
	float velocity_;		
	float current_;
	float acceleration_; 		
	float time_required_;
};  
