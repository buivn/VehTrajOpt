/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MP_SIMULATOR_HPP_
#define GP__MP_SIMULATOR_HPP_

#include "GP/SceneCells.hpp"
#include "Utils/Params.hpp"
#include "Utils/Allocator.hpp"
#include "Utils/Algebra.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Flags.hpp"

namespace GP
{
    class MPSimulator
    {
    public:
	MPSimulator(void);
		
	virtual ~MPSimulator(void);

	virtual void SetupFromParams(Params * const p);
	
	virtual void CompleteSetup(void);
	
	virtual void SetScene(SceneCells *scene)
	{
	    m_scene = scene;
	}
	
	virtual SceneCells* GetScene(void)
	{
	    return m_scene;
	}
	
	virtual Allocator<double>* GetStateAllocator(void)
	{
	    return &m_allocatorState;
	}
	
	virtual Allocator<double>* GetControlAllocator(void)
	{
	    return &m_allocatorControl;
	}

	virtual void SetTimeStep(const double dt)
	{
	    m_dt = dt;
	}
	
	virtual double GetTimeStep(void) const
	{
	    return m_dt;
	}

	virtual double GetMinDistOneStep(void) const
	{
	    return m_minDistOneStep;
	}
	
	virtual double GetMaxDistOneStep(void) const
	{
	    return m_maxDistOneStep;
	}
	
	virtual void SetMinDistOneStep(const double d)
	{
	    m_minDistOneStep = d;
	}
	
	virtual void SetMaxDistOneStep(const double d)
	{
	    m_maxDistOneStep = d;
	}

	virtual const double* GetState(void) const
	{
	    return m_currState;
	}
	
	virtual bool SampleValidStateNearPos(double s[], const double r,  const double scale, const int nrTries) = 0;
	
	
	virtual void SetState(const double s[]);
	virtual void GetState(double s[]);
	virtual void SampleState(double s[]) = 0;
	virtual void ClearNonCfgInState(double s[]) = 0;	
	virtual bool IsStateValid(void) = 0;
	virtual double DistanceStates(const double s1[], const double s2[]) = 0;
  
	virtual double EuclideanDistanceStates(const double s1[], const double s2[])
	{
	    //return Algebra::PointDist(m_allocatorState.GetDim(), s1, s2);
	    const double da = Algebra2D::AngleDist(s1[2], s2[2]);
	    const double dd = Algebra2D::PointDist(s1, s2);
	    
	    return dd + 2 * da;
	    
	}
	
  
	virtual void SetControl(const double u[]);
	virtual void GetControl(double u[]);
	virtual void SampleControl(void) = 0;
	virtual void StartSteerToPosition(const double target[]) = 0;
	virtual void SteerToPosition(const double target[]) = 0;
	virtual bool HasReachedSteerPosition(const double target[], const double dtol) = 0;
			
	virtual void SimulateOneStep(void) = 0;

	virtual void Draw(void)
	{
	}
	
	virtual void DrawSimple(void)
	{
	}
	
	
    protected:
	virtual double GetVelocity(void)
	{
	    return 0;
	}
	
	virtual double GetAcceleration(void)
	{
	    return 0;
	}
	
	virtual double TimeStepConstantVelocity(const double v, const double d) const;
	
	virtual double TimeStepConstantAcceleration(const double v, 
						    const double a, 
						    const double d) const;  
	SceneCells            *m_scene;
	Allocator<double> m_allocatorState;
	Allocator<double> m_allocatorControl;
	double            m_dt;
	double            m_minDistOneStep;
	double            m_maxDistOneStep;
	double           *m_currState;
	double           *m_currControl;
	
    };    
}

#endif


