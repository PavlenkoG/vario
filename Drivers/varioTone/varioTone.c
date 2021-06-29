/*
 * varioTone.c
 *
 *  Created on: 28 Jan 2020
 *      Author: GRPA
 */

#include <varioTone.h>

// climb speed in m/s
double varioSpeed[VARIO_TABLE_SIZE] = {
//      0      1     2     3     4      5      6     7  8    9     10   11   12   13   14
        -10.0, -5.0, -3.0, -2.0, -1.99, -0.51, -0.5, 0, 0.5, 0.51, 1.0, 2.0, 3.0, 5.0, 10.0
};

// vario tone frequency in Hz
uint32_t varioFreq[VARIO_TABLE_SIZE] = {
//      0    1    2    3    4    5    6    7    8    9    10   11   12    13    14
        200, 225, 275, 350, 351, 549, 550, 650, 750, 771, 825, 950, 1060, 1250, 1600
};
// vario cycle in ms
uint32_t varioCycle[VARIO_TABLE_SIZE] = {
//      0    1    2    3    4     5     6     7     8    9    10   11   12   13   14
        100, 100, 100, 100, 3000, 2000, 1500, 1000, 800, 500, 450, 400, 350, 200, 150
//		1000,1000,1000,1000,1000, 1000, 1000, 1000, 1000,1000,1000,1000,1000,1000,1000
};
// vario duty cycle in %
uint32_t varioDuty[VARIO_TABLE_SIZE] = {
//      0    1    2    3    4    5  6   7    8    9   10  11  12  13  14
        100, 100, 100, 100, 10, 15, 20, 25,  50,  55, 60, 63, 65, 68, 70
};


void getVarioTone(double climbSpeed, struct varioTone* pTone) {
    uint8_t index = 0;
    double offsetPercent = 0.0;

    uint32_t toneFreq = 0;
    // find the table index
    if (climbSpeed <= varioSpeed[0]) {
        pTone->toneFreq = varioFreq[0];
        pTone->cycle = varioCycle[0];
        pTone->toneDutyCycle = varioDuty[0];
    }
    if (climbSpeed >= varioSpeed[VARIO_TABLE_SIZE-1]) {
        pTone->toneFreq = varioFreq[VARIO_TABLE_SIZE-1];
        pTone->cycle = varioCycle[VARIO_TABLE_SIZE-1];
        pTone->toneDutyCycle = varioDuty[VARIO_TABLE_SIZE-1];
    }
    //sarching for current table index
    if (climbSpeed > varioSpeed[0]&&climbSpeed <varioSpeed[VARIO_TABLE_SIZE-1]) {
        while (climbSpeed > varioSpeed[index]) {
            index ++;
        }
    }
    if (index && index < VARIO_TABLE_SIZE) {
        // tone frequency calculation
        pTone->toneFreq = getLinearInterpolation(&varioFreq[0], index, climbSpeed);

        // cycle calculation
        pTone->cycle = getLinearInterpolation(&varioCycle[0], index, climbSpeed);

        // duty cycle calculation
        pTone->toneDutyCycle = getLinearInterpolation(&varioDuty[0], index, climbSpeed);

    }
    // printf("index = %d; speed = %f; freq = %d; cycle = %d; duty = %d\r\n",index, climbSpeed,  pTone->toneFreq, pTone->cycle, pTone->toneDutyCycle);
    // struct timerSound ts = getTimerSoundConfig(pTone);
    // printf("cycCnt = %d; onCnt = %d; offCnt = %d\r\n", ts.cyclCount, ts.onCount, ts.offCount);
}

double getVarioSpeed (uint8_t index) {
    return varioSpeed[index];
}

struct timerSound getTimerSoundConfig (struct varioTone* pTone) {
	struct timerSound timersound;
	timersound.cyclCount = (48000000/4*0.001) * pTone->cycle;
	timersound.onCount = (timersound.cyclCount / 100) * (pTone->toneDutyCycle);
	timersound.offCount = timersound.cyclCount - timersound.onCount;
	return timersound;
}

uint32_t getLinearInterpolation (uint32_t *calcArray, uint8_t index, double speed) {
	//f(x0) + (f(x1) - f(x0))/(x1-x0) * (x-x0)
	double dfx0 = (double)(calcArray[index-1]);
	double dfx1 = (double)(calcArray[index]);
	double dx0 = (double)(varioSpeed[index-1]);
	double dx1 = (double)(varioSpeed[index]);
	return (unsigned int) (dfx0 + (dfx1-dfx0)/(dx1-dx0)*(speed-dx0));
}
