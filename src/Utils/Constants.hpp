/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__CONSTANTS_HPP_
#define GP__CONSTANTS_HPP_

#include "Utils/Definitions.hpp"
#include <cmath>

namespace GP
{
    namespace Constants
    {
	enum
	    {
		NEITHER_PARALLEL_NOR_ANTI_PARALLEL = 0,
		PARALLEL                           = 1,
		ANTI_PARALLEL                      = 2
	    };

	
	const double EPSILON         = ldexp(1.0, -36);
	const double EPSILON_SQUARED = ldexp(1.0, -72);
	const double SQRT_EPSILON    = ldexp(1.0, -18);
	
	const double RAD2DEG         = 180 / M_PI;
	const double DEG2RAD         = M_PI / 180;

	const int ID_UNDEFINED       = -1;

	const int GRAPHICS_RESOLUTION_X = 1280;
	const int GRAPHICS_RESOLUTION_Y =  720;
	const int GRAPHICS_ONTIMER_INTERVAL = 2;
	
	
    }
}

#endif
    
    
    
    







