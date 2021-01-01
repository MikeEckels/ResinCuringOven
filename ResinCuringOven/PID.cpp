#include "PID.h"

PID::PID() {
	this->kp = 1;
	this->ki = 0;
	this->kd = 0;
}

PID::~PID() {

}

PID::PID(float kp, float ki, float kd) {
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

float PID::Calculate(float setpoint, float processVariable, float dT) {
	float POut, IOut, DOut;

	error = setpoint - processVariable;
	integral += error * dT;

	POut = kp * error;
	IOut = ki * integral;
	DOut = kd * ((error - previousError) / dT);

	output = POut + IOut + DOut;
	previousError = error;

	return output;
}
