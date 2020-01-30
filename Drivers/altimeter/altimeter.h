#include "stm32f0xx_hal.h"

typedef struct tSimpleKalman {
	double KG;		// initial Kalman gain
	double Emea;	// error in measure
	double Eest;	// error in estimate
	double ESTt;	// current estimate
	double ESTt0;   // previous estimate
} SimpleKalman;

typedef struct tAltimeterHight {
	double actualHight;
	double lastHight;
} altimeterHight;

void initKalman (SimpleKalman *kalman);
double simpleKalman (SimpleKalman *kalman, float measValue);

double getVerticalSpeed (altimeterHight *hight, uint32_t dTime);
