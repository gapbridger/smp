#include"Arm01.h"
#include<iostream>
#include<math.h>
#define L1	0.3770
#define L2	0.5736
using namespace std; 

void stLine(){}; 

void main(){}; 

float XY2Theta(float theta1, float theta2)
{
	
}
float Theta2XY(float theta1, float theta2)
{
	float ret[2] = [0, 0];
	float xCor, yCor; 
	xCor = sin(theta1)*L1+sin(theta2-theta1)*L2; 
	yCor = cos(theta1)*L1+cos(theta2-theta1)*L2; 
	ret[0] = xCor; 
	ret[1] = yCor; 
}
