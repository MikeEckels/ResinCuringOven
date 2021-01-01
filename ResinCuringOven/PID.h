#pragma once

class PID {
private:
	float integral = 0;
	float error = 0;
	float previousError = 0;
	float output = 0;
	float kp;
	float ki;
	float kd;

public:
	PID();
	~PID();
	PID(float kp, float ki, float kd);
	float Calculate(float setpoint, float processVariable, float dT);
};
