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
//      0   1   2   3   4   5   6   7   8   9    10   11   12  13  14
        10, 10, 10, 10, 10, 10, 10, 10, 300, 300, 300, 125, 90, 70, 50
};
// vario duty cycle in %
uint8_t varioDuty[VARIO_TABLE_SIZE] = {
//      0    1    2    3    4    5    6    7    8    9   10  11  12  13  14
        100, 100, 100, 100, 100, 100, 10,  20,  30,  50, 50, 50, 50, 50, 50
};

void getVarioTone(double climbSpeed, struct varioTone* tone) {
    uint8_t index = 0;
    double offsetPercent = 0.0;
    double climbDiff = 0.0;
    double tableDiff = 0.0;
    double percent = 0.0;

    uint32_t toneFreq = 0;
    // find the table index
    if (climbSpeed <= varioSpeed[0]) {
        tone->toneFreq = varioFreq[0];
        tone->cycle = varioCycle[0];
        tone->toneDutyCycle = varioDuty[0];
    }
    if (climbSpeed >= varioSpeed[VARIO_TABLE_SIZE-1]) {
        tone->toneFreq = varioFreq[VARIO_TABLE_SIZE-1];
        tone->cycle = varioCycle[VARIO_TABLE_SIZE-1];
        tone->toneDutyCycle = varioDuty[VARIO_TABLE_SIZE-1];
    }
    if (climbSpeed > varioSpeed[0]&&climbSpeed <varioSpeed[VARIO_TABLE_SIZE-1]) {
        while (climbSpeed > varioSpeed[index]) {
            index ++;
        }
    }
    if (index && index < VARIO_TABLE_SIZE) {
        climbDiff = fabs(fabs(climbSpeed) - fabs(varioSpeed[index]));
        tableDiff = fabs(fabs(varioSpeed[index]) - fabs(varioSpeed[index-1]));
        if (tableDiff != 0) {
            percent = climbDiff/tableDiff;
        }
        // tone frequency calculation
        tone->toneFreq = (unsigned int)((varioFreq[index]-varioFreq[index-1])*(1-percent))+varioFreq[index-1];

        // cycle calculation
        if (varioCycle[index-1]>varioCycle[index]){
            tone->cycle =(unsigned int) ((double)(varioCycle[index])+((double)((abs(varioCycle[index]-varioCycle[index-1])))*(1-percent)));
        }else {
            tone->cycle =(unsigned int) ((double)(varioCycle[index-1])-((double)((abs(varioCycle[index]-varioCycle[index-1])))*(1-percent)));
        }

        // duty cycle calculation
        if (varioDuty[index-1]>varioDuty[index]){
            tone->toneDutyCycle =(unsigned int) ((double)(varioDuty[index])+((double)((abs(varioDuty[index]-varioDuty[index-1])))*(1-percent)));
        }else {
            tone->toneDutyCycle =(unsigned int) ((double)(varioDuty[index-1])-((double)((abs(varioDuty[index]-varioDuty[index-1])))*(1-percent)));
        }

    }

    printf("speed = %f; percent = %f; freq = %d; cycle = %d; duty = %d\r\n", climbSpeed, percent*100, tone->toneFreq, tone->cycle, tone->toneDutyCycle);

}

double getVarioSpeed (uint8_t index) {
    return varioSpeed[index];
}

uint32_t onTime (struct varioTone* tone) {
	return (((48000000/4*0.001) * tone->cycle)/100) * tone->toneDutyCycle;
}
uint32_t offTime (struct varioTone * tone) {
	return getCycle - onTime(&tone);
}
uint32_t getCycle (struct varioTone* tone) {
	return (48000000/4*0.001) * tone->cycle;
}
uint32_t getDutyCycle (struct varioTone* tone) {
	return tone->toneDutyCycle;
}
