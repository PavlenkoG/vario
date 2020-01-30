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
};


void getVarioTone(double climbSpeed, struct varioTone* tone);
double getVarioSpeed (uint8_t index);


#endif /* DRIVERS_VARIOTONE_VARIOTONE_H_ */
