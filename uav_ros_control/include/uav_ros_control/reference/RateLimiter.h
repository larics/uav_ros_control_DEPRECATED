#ifndef RATE_LIMITER_H
#define RATE_LIMITER_H

#include <iostream>

class RateLimiter {
/* Class implementation of Rate Limiter. */
private:
	float m_rate, m_input, m_sampleTime, m_lastOutput, m_output;
	float m_R, m_F;
public:
	//RateLimiter(float t_sampleTime, float t_R, float t_F, float t_input);
	RateLimiter(void);
	~RateLimiter(void);
	void init(float t_sampleTime, float t_R, float t_F, float t_input);
	void reset();
	void initialCondition(float t_input, float t_offset);
	void setInput(float t_data);
	void setSampleTime(float t_data);
	void setR(float t_data);
	void setF(float t_data);
	float getData(void);

};

#endif //RATE_LIMITER_H