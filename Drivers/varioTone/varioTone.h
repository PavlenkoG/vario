/*
 * varioTone.h
 *
 *  Created on: 28 Jan 2020
 *      Author: GRPA
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef DRIVERS_VARIOTONE_VARIOTONE_H_
#define DRIVERS_VARIOTONE_VARIOTONE_H_

#define VARIO_TABLE_SIZE    15

struct varioTone {
    uint32_t        toneFreq;
    uint32_t        cycle;
    uint32_t        toneDutyCycle;
    double			averageSpeed;
};

struct timerSound {
	uint32_t		cyclCount;
	uint32_t		onCount;
	uint32_t		offCount;
};


void getVarioTone(double climbSpeed, struct varioTone* pTone);
double getVarioSpeed (uint8_t index);
uint32_t getLinearInterpolation (uint32_t *calcArray, uint8_t index, double speed);
struct timerSound getTimerSoundConfig (struct varioTone* pTone);

uint32_t onTime (struct varioTone* pTone);
uint32_t offTime (struct varioTone* pTone);
uint32_t getCycle (struct varioTone* pTone);
uint32_t getDutyCycle (struct varioTone* pTone);


#endif /* DRIVERS_VARIOTONE_VARIOTONE_H_ */
