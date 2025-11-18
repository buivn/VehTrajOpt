/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__FLAGS_HPP_
#define GP__FLAGS_HPP_

#include "Utils/Definitions.hpp"

namespace GP
{
    typedef unsigned int Flags;
   
    static inline
    bool HasFlag(const Flags f, const Flags b)
    {
	return f & b;
    }
          
    static inline
    Flags AddFlag(const Flags f, const Flags b)
    {
	return f | b;
    }
        

    static inline
    Flags RemoveFlag(const Flags f, const Flags b)
    {
	return f & (~b);
    }
    
    static inline
    Flags FlipFlag(const Flags f, const Flags b)
    {
	return f ^ b;
    }
}

#endif
    
    
    
    







