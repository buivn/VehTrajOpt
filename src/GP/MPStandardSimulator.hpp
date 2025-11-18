/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MP_STANDARD_SIMULATOR_HPP_
#define GP__MP_STANDARD_SIMULATOR_HPP_

#include "GP/MPSimulator.hpp"

namespace GP
{
    class MPStandardSimulator : public MPSimulator
    {
    public:
	MPStandardSimulator(void) : MPSimulator()
	{
	}
	
		
	virtual ~MPStandardSimulator(void)
	{
	}
	
	
	virtual void SimulateOneStep(void);

    protected:
	virtual void MotionEqs(const double s[],
			       const double t,
			       const double u[],
			       double ds[]) = 0;
    };    
}

#endif


