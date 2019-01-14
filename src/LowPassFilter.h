#ifndef _LowPassFilter_h_
#define _LowPassFilter_h_

#include <iostream>
#include <cmath>

class LowPassFilter{
public:
	//constructors
	LowPassFilter();
	LowPassFilter(float iCutOffFrequency);
	LowPassFilter(float iCutOffFrequency, float iDeltaTime);
	//functions
	float update(float input);
	float update(float input, float deltaTime);
	//get and set funtions
	float getOutput();
	float getCutOffFrequency();
	void setCutOffFrequency(float input);
	void setDeltaTime(float input);
private:
	float output;
	float cutOffFrequency;
	float ePow;
};

#endif //_LowPassFilter_h_