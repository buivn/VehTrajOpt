/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/MPSimulator.hpp"
#include "Utils/Misc.hpp"
#include "GP/GPConstants.hpp"

namespace GP
{
    MPSimulator::MPSimulator(void)
    {
	m_dt             = Constants::SIMULATOR_TIME_STEP;
	m_minDistOneStep = Constants::SIMULATOR_MIN_DISTANCE_ONE_STEP;
	m_maxDistOneStep = Constants::SIMULATOR_MAX_DISTANCE_ONE_STEP;
	m_scene          = NULL;
	m_currState      = NULL;
	m_currControl    = NULL;
	
   }
	
    MPSimulator::~MPSimulator(void)
    {
	m_allocatorState.Delete(m_currState);
	m_allocatorControl.Delete(m_currControl);
    }

    void MPSimulator::SetupFromParams(Params * const p)
    {
	m_dt             = p->GetValueAsDouble("SimulatorTimeStep", m_dt);
	m_minDistOneStep = p->GetValueAsDouble("SimulatorMinDistanceOneStep", m_minDistOneStep);
	m_maxDistOneStep = p->GetValueAsDouble("SimulatorMaxDistanceOneStep", m_maxDistOneStep);
    }
    
    void MPSimulator::CompleteSetup(void)
    {
	m_currState = m_allocatorState.New();
	m_currControl = m_allocatorControl.New();

	m_allocatorState.Fill(m_currState, 0.0);
	m_allocatorControl.Fill(m_currControl, 0.0);
    }
    

    void MPSimulator::SetState(const double s[])
    {
	if(s != m_currState)
	    m_allocatorState.Copy(m_currState, s);
    }

    void MPSimulator::GetState(double s[])
    {
	m_allocatorState.Copy(s, m_currState);
    }


    void MPSimulator::SetControl(const double u[])
    {
	m_allocatorControl.Copy(m_currControl, u);
    }

    void MPSimulator::GetControl(double u[])
    {
	m_allocatorControl.Copy(u, m_currControl);
    }    
  	
    double MPSimulator::TimeStepConstantVelocity(const double v, const double d) const
    {
	const double absv = fabs(v);
	
	return absv < Constants::EPSILON ? (d / Constants::EPSILON) : (d / absv);	    
    }
    
    double MPSimulator::TimeStepConstantAcceleration(const double v, 
						     const double a, 
						     const double d) const
    {
	if(fabs(a) < Constants::EPSILON)
	    return TimeStepConstantVelocity(v, d);
	
	if(a >= 0.0 && v >= 0.0)
	    return fabs((-v + sqrt(v * v + 2 * a * d)) / a);
	else if(a <= 0.0 && v <= 0.0)
	    return fabs((v + sqrt(fabs(v * v - 2 * a * d))) / (-a));    
	else if(a <= 0.0 && v >= 0.0)
	{
	    const double b2_4ac = v * v + 2 * a * d;
	    if(b2_4ac >= 0.0)
		return fabs((-v + sqrt(b2_4ac)) / a);
	    const double dist_remaining = d - v * v / (2*a);
	    //traveled from speed zero in reverse
	    return sqrt(2 * dist_remaining / fabs(a));
	}
	else if(a >= 0 && v <= 0.0)
	{
	    const double b2_4ac = v * v - 2 * a * d;
	    if(b2_4ac >= 0.0)
		return fabs((-v - sqrt(b2_4ac)) / a);	
	    const double dist_remaining = d + v * v / (2 * a);
	    return sqrt(2 * dist_remaining / a);	
	}
	
	return 0.0;    
    }   
}

