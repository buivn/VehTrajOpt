/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MP_STRAILER_SIMULATOR_HPP_
#define GP__MP_STRAILER_SIMULATOR_HPP_

#include "GP/MPStandardSimulator.hpp"
#include "Utils/PIDController.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Car.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include <vector>

namespace GP
{
    class MPSnakeSimulator : public MPStandardSimulator
    {
    public:
	MPSnakeSimulator(void);
	
			
	virtual ~MPSnakeSimulator(void)
	{
	}
		
	enum 
	    {
		STATE_X           = 0,
		STATE_Y           = 1,
		STATE_THETA       = 2,
		STATE_VEL         = 3,
		STATE_STEER_ANGLE = 4,
		STATE_LINKS       = 5,
		CONTROL_ACC       = 0,
		CONTROL_STEER_VEL = 1
	    };

	virtual void SetupFromParams(Params * const p);
		
	virtual double GetVelocity(void)
	{
	    return m_currState[STATE_VEL];
	}
	
	virtual double GetAcceleration(void)
	{
	    return m_currControl[CONTROL_ACC];
	}

	virtual double DistanceStates(const double s1[], const double s2[])
	{
	    return Algebra2D::PointDist(s1, s2);
	}
	
 
	virtual void CompleteSetup(void);

	virtual void SetState(const double s[]);

	virtual void SampleState(double s[])
	{
	    s[STATE_X] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[0], 
					   GetScene()->GetGrid()->GetMax()[0]);
	    s[STATE_Y] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[1], 
					   GetScene()->GetGrid()->GetMax()[1]);
	    s[STATE_THETA] = RandomUniformReal(-M_PI, M_PI);
	    s[STATE_VEL] = RandomUniformReal(m_minVel, m_maxVel);
	    s[STATE_STEER_ANGLE] = RandomUniformReal(m_minSteerAngle, m_maxSteerAngle);
	      for(int i = 0; i < m_nrLinks; ++i)
		  s[STATE_LINKS + i] = 0;//RandomUniformReal(-M_PI, M_PI);
	    
	    
	}

	// question frontier to frontier: x,y position for initial must be set, and draw a circle
	// other state elements can be set
	// play with r??????? 


	virtual bool SampleValidStateNearPos(double s[], const double r, const double scale,  const int nrTries)
	{
	    const double c[] = {s[STATE_X], s[STATE_Y]};
	    
	    s[STATE_VEL] = 0.0;
	    s[STATE_STEER_ANGLE] = 0.0;
	    
	    for(int i = 0; i < nrTries; ++i)
	    {
		s[STATE_THETA] = RandomUniformReal(-M_PI, M_PI);
		for(int i = 0; i < m_nrLinks; ++i)
		    s[STATE_LINKS + i] = RandomUniformReal(-scale*M_PI, scale*M_PI);
		SetState(s);
		if(IsStateValid())
		    return true;
		SampleRandomPointInsideCircle2D(c, r, &s[STATE_X]);
	    }
	    return false;
	    
	}
	

	
	virtual void ClearNonCfgInState(double s[])
	{
	    s[STATE_VEL] = 0;
	    s[STATE_STEER_ANGLE] = 0;
	    for(int i = 0; i < m_nrLinks; ++i)
		  s[STATE_LINKS + i] = 0;//RandomUniformReal(-M_PI, M_PI);

	}
	
	
	virtual bool IsStateValid(void);
	
	virtual void StartSteerToPosition(const double target[]);
	virtual void SteerToPosition(const double target[]);
	virtual bool HasReachedSteerPosition(const double target[], const double dtol);

	virtual void SampleControl(void)
	{
	    m_currControl[CONTROL_ACC] = RandomUniformReal(m_minAcc, m_maxAcc);
	    m_currControl[CONTROL_STEER_VEL] = RandomUniformReal(m_minSteerVel, m_maxSteerVel);
	}
	
		

	virtual void Draw(void);

	virtual void DrawSimple(void);
	
	    

	bool   m_frontWheelDriving;	    
	double m_bodyLength;
	double m_bodyWidth;
	int    m_nrLinks;
	double m_attachDist;
	double m_minSteerAngle;
	double m_maxSteerAngle;
	double m_minVel;
	double m_maxVel;
	double m_minAcc;
	double m_maxAcc;
	double m_minSteerVel;
	double m_maxSteerVel;

	
	virtual void ConstructTriMesh(void);
	
	virtual void MotionEqs(const double s[],
			       const double t,
			       const double u[],
			       double ds[]);

	std::vector<double> m_bodies;
	double              m_orig[8];
	std::vector<double> m_TRs;
	PIDController       m_pidSteer;
	PIDController       m_pidVel;
	PQPTriMesh          m_tmeshSnake;
	//Car                 m_car;
	
    };    
}

#endif


