#include <uav_ros_control/reference/RateLimiter.h>

RateLimiter::RateLimiter()
{
	// m_sampleTime = t_sampleTime;
	// m_R = t_R;
	// m_F = t_F;
	// m_input = t_input;
}

RateLimiter::~RateLimiter(void)
{
}

void RateLimiter::init(float t_sampleTime, float t_R, float t_F, float t_input)
{
	m_sampleTime = t_sampleTime;
	m_R = t_R;
	m_F = t_F;
	m_input = t_input;	
}

void RateLimiter::reset(){
	m_input = 0.00001;
	m_lastOutput = 0.0;
}

void RateLimiter::initialCondition(float t_input, float t_offset){
	m_input = t_input;
	m_lastOutput = t_offset;
}

void RateLimiter::setInput(float t_data)
{
	m_input = t_data;
}

void RateLimiter::setSampleTime(float t_data)
{
	m_sampleTime = t_data;
}

void RateLimiter::setR(float t_data)
{
	m_R = t_data;
}

void RateLimiter::setF(float t_data)
{
	m_F = t_data;
}

float RateLimiter::getData(void)
{
	m_rate = (m_input - m_lastOutput) / (m_sampleTime);

	if (m_rate > m_R){
		m_output = m_sampleTime * m_R + m_lastOutput;
	}
	else if (m_rate < m_F){
		m_output = m_sampleTime * m_F + m_lastOutput;
	}
	else {
		m_output = m_input;
	}

	m_lastOutput = m_output;

	return m_output;
}