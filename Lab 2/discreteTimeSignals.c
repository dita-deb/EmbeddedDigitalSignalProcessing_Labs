/*#******************************************
#
#  Author: Coskun Tekes, PhD
#  Course:  CPE3500 - Embedded Digital Signal Processing
#
#  Created on: Jan 29, 2024
#
#  Description: This sample code generates basic discrete time 
#	        signals to be used in STM32CubeIde
#
#*******************************************/

#include "math.h"
#include "arm_math.h"

#define N 20

int n;
int delta[N], step[N], rect[N];
float expon[N], sinus[N];
float a=0.8, w0=PI/4;

int main()
{
  // unit impulse signal
  for(n=0;n<N;n++)
	  if (n==0) delta[n]=1;
	  	  else delta[n]=0;
	  	  
  //unit step signal
  for(n=0;n<N;n++)
	  step[n]=1;

   //rectangular signal between n=0 and 5
  for(n=0;n<N;n++)
	  if ((n>=0) & (n<6))
		  rect[n]=1;
	  else
		  rect[n]=0;

  //exponential signal
  for(n=0;n<N;n++)
	  expon[n]=pow(a,(float)n);

  //sinusoidal signal
  for(n=0;n<N;n++)
	  sinus[n]=sin(w0*(float)n);

   return 0;
}

