#include "ntcan.h"

using namespace std; 

#define TRUE_REFERENCED		0x01     // |
#define TRUE_MOVING			0x02
#define TRUE_ERROR			0x10
#define TRUE_BRAKE			0x20
#define TRUE_MOV_END		0x40
#define TRUE_POS_REACHED	0x80

#define FALSE_REFERENCED	0xFE    // &
#define FALSE_MOVING		0xFD
#define FALSE_ERROR			0xEF
#define FALSE_BRAKE			0xDF
#define FALSE_MOV_END		0xBF
#define FALSE_POS_REACHED	0x7F

class MODULE
{
public: 
	MODULE(); 
	~MODULE(); 
	void	setId(int32_t _id); 
	void	setStatusCode(uint8_t _statusCode); 
	void	setErrorCode(uint8_t errorCode); 
	void	setMode(uint8_t _mode); 
	void	updatePos(float _pos); 
	void    updateVel(float _vel); 
	void	updateAcc(float _acc); 
	void	updateCur(float _cur); 

	int32_t	getId(); 
	uint8_t getStatusCode(); 
	uint8_t	getErrorCode(); 
	uint8_t	getMode();    // mode: para. in CMD_GET_STATE
	float	getPos(); 
	float	getVel(); 
	float	getAcc(); 
	float	getCur(); 

	bool	checkReferenced();
	bool	checkMoving(); 
	bool	checkError(); 
	bool	checkBrake(); 
	bool	checkMoveEnd(); 
	bool	checkPosReached(); 

private:
	int32_t	id;				
	uint8_t	statusCode;			// Byte of status 8-bit 
	uint8_t	errorCode;		// Byte of error type
	uint8_t	mode;

	float	pos;		
	float	vel;		
	float	cur;
	float	acc; 		
};  