/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/Region.hpp"
#include "GP/Decomposition.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"

namespace GP
{
    void Region::SetWeightBasedOnPathCost(void)
    {
	const double h   = GetPathDataToGoal()->m_cost;
	if(h == HUGE_VAL)
	    SetWeight(Constants::EPSILON);
	else
	    SetWeight(1000.0 / h); // pow(1.0 / h, hexp));

	
	// SetWeight(100000000 * pow(1 - h/m_decomp->GetMaxCostPathToGoal(), hexp));
    }
    
    void Region::AddNeighbor(const int id, const double cost)
    {
//	printf("adding %d neighbor %d with cost %f\n", GetId(), id, cost);
	
	const int pos = FindItem<int>(&m_neighs, id);
	if(pos >= 0)
	    m_costs[pos] = cost;
	else
	{
	    m_neighs.push_back(id);
	    m_costs.push_back(cost);
	}
    }	
}
