#include "altimeter.h"

void initKalman (SimpleKalman *kalman) {
	kalman->KG = 0.0;
	kalman->Emea = 0.7;
	kalman->Eest = 0.01;
	kalman->ESTt = 0;
	kalman->ESTt0 = 0;
}

double simpleKalman (SimpleKalman *kalman, float measValue) {
	// calculate Kalman gain
	kalman->KG = kalman->Eest/(kalman->Eest + kalman->Emea);
	// calculate current estimate
	kalman->ESTt = kalman->ESTt0 + kalman->KG*(measValue - kalman->ESTt0);
	kalman->ESTt0 = kalman->ESTt;
	return kalman->ESTt;
}

double getVerticalSpeed (altimeterHight *hight, uint32_t dTime) {
	double verticalSpeed = 0.0;
	double dTimeFloat = dTime/1000.0; // delta time in seconds
	if (hight->lastHight != 0.0) {
		verticalSpeed = (hight->actualHight - hight->lastHight)/dTimeFloat;
	}
	hight->lastHight = hight->actualHight;
	return verticalSpeed;
}
