/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/Goal.hpp"
#include "Utils/PrintMsg.hpp"
#include <cstdio>

namespace GP
{
    void Goal::Read(const char fname[])
    {
	FILE *in = fopen(fname, "r");
	if(!in)
	{
	    OnInputError(printf("..could not open <%s> for reading\n", fname));
	    return;
	}
	
	m_poly.Read(in);
	
	fclose(in);
	
    }
}



