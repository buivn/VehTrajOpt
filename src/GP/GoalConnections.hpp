/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP_GOAL_CONNECTIONS_HPP_
#define GP_GOAL_CONNECTIONS_HPP_

#include <vector>

namespace GP
{
    class GoalConnections
    {
    public:
	GoalConnections(void)
	{
	}
	
	virtual ~GoalConnections(void)
	{
	}
	
	virtual void Clear(void)
	{
	    m_neighs.clear();
	    m_costs.clear();
	}

	virtual int GetNrNeighbors(void)
	{
	    return m_neighs.size();
	}
	
	virtual const int* GetNeighbors(void)
	{
	    return &m_neighs[0];
	}
	
	virtual const double* GetCostsNeighbors(void)
	{
	    return &m_costs[0];
	}
	
	virtual void AddNeighbor(const int rid, const double c)
	{
	    m_neighs.push_back(rid);
	    m_costs.push_back(c);
	}

	virtual int FindNeighbor(const int rid) const
	{
	    return FindItem<int>(&m_neighs, rid);
	    
	}
	

    protected:
	std::vector<int>    m_neighs;
	std::vector<double> m_costs;
	
	
    };   

}

#endif



