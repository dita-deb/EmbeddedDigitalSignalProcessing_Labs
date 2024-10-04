/*#******************************************
#
#  Author: Coskun Tekes, PhD
#  Course:  CPE3500 - Embedded Digital Signal Processing
#
#  Created on: Feb 5, 2024
#
#  Description: This sample code generates a DT system which calculates convolutions sum  
#	        to filter composite sinusoildal signal to be used in STM32CubeIde (Lab-3)
#
#*******************************************/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"

/* USER CODE END Includes */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 41
#define FILTER_SIZE 15
/* USER CODE END PD */



/* USER CODE BEGIN PV */

static float filter[FILTER_SIZE] = {     // Discrete time low pass filter which filters above 20 Hz
  -0.01259277,
  -0.02704833,
  -0.03115702,
  -0.00335167,
  0.06651710,
  0.16356430,
  0.24972947,
  0.28427791,
  0.24972947,
  0.16356430,
  0.06651710,
  -0.00335167,
  -0.03115702,
  -0.02704833,
  -0.01259277
};

int n;
float sine1[N], sineComposite[N], sineFiltered[N];


float f1=5; // Sine Frequency 1
float f2=5; // Sine Frequency 2
float f3=5; // Sine Frequency 3
float f4=5; // Sine Frequency 4
float fs=100;   // Sampling Frequency

/* USER CODE END PV */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// DT Convolution Sum Calcaulation for x->input, h->impulse respnse, y->output, sizeH->length of h
void convolution(float x[], float h[], float y[], int sizeX, int sizeH)
{
    int n, k;  // variables for loops
    int p=0;    // index variable for output signal y
    float sum;  // convolutions sum accumulator variable
    int Low_lim = sizeH/2;      // Staring point of the center pick N elements of the output array
	int L = sizeX + sizeH - 1;  // Original length of the convolution.

	for (n = 0; n < L; n++) {
		sum = 0;
		for (k = 0; k < sizeX; k++) {

			if ((n - k) >= 0 && (n - k) < sizeH) {
				
				sum = sum + x[k]*h[n-k];   // Main convolution sum calculation
			}

		}
		if(n>=Low_lim && n<(Low_lim+N))  // Only pick center N elements of the output array!
		{
		    y[p] = sum;
	        p++;
		}
	}
}

/* USER CODE END 0 */

int main(void)
{
  


  /* USER CODE BEGIN 2 */

  //Single Tone Sinusoidal signal and Four Frequency Combined Sinusoidal Signal
  for(n=0;n<N;n++)
  {
	  sine1[n]=sin(2*PI*f1/fs*(float)n);

	  sineComposite[n]=sine1[n]+sin(2*PI*f2/fs*(float)n)+sin(2*PI*f3/fs*(float)n)+sin(2*PI*f4/fs*(float)n);

  }
  // Call Convolution Function to filter Combined Signal
  convolution(sineComposite, filter, sineFiltered, N, FILTER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


