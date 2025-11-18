/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__SOLUTION_HPP_
#define GP__SOLUTION_HPP_

#include <vector>
#include <cstdio>

namespace GP
{
    class Solution
    {
    public:
	Solution(void)
	{
	    Clear();
	}

	virtual ~Solution(void)
	{
	}

	virtual void Clear(void)
	{
	    m_dimState = 1;
	    m_trajStates.clear();
	    m_trajCostOverall = m_trajCostFreeSpace = 0.0;

	    m_remainingPathPoints.clear();
	    m_remainingPathCostOverall = m_remainingPathCostFreeSpace = 0.0;

	    m_goalX = m_goalY = m_goalRadius = 0.0;

	    m_solutionFound = true;	    
	}

	virtual bool SolutionFound(void) const
	{
	    return m_solutionFound;
	}
	
	virtual int TrajGetNrStates(void) const
	{
	    return m_trajStates.size()  / m_dimState;
	}

	virtual const double* TrajGetState(const int i) const
	{
	    return &m_trajStates[i * m_dimState];	    
	}

	virtual int RemainingPathGetNrPoints(void) const
	{
	    return m_remainingPathPoints.size() / 2;	    
	}

	virtual const double* RemainingPathGetPoint(const int i) const
	{
	    return &m_remainingPathPoints[2 * i];
	}

	virtual void PrintInfo(void) const
	{
	    printf("Trajectory:    NrStates = %4d CostOverall = %6.2f CostFree = %6.2f\n", TrajGetNrStates(), m_trajCostOverall, m_trajCostFreeSpace);
	    printf("RemainingPath: NrPoints = %4d CostOverall = %6.2f CostFree = %6.2f\n", RemainingPathGetNrPoints(), m_remainingPathCostOverall, m_remainingPathCostFreeSpace);
	}

	//Variable to indicate whether or not a solution has been found
	bool m_solutionFound;
	
	//Number of state dimensions. It is set to 5 + nrLinks, where nrLinks is the number of links in the snake model.
	//For example, a snake with 5 links would have a state with 10 dimensions
	int m_dimState;

	//Solution trajectory. All states stored in one-dimensional vector.
	//State i is stored in positions dimState*i ...dimState*(i+1) - 1
	//You can use the functions TrajGetNrStates to figure out how many states are in the trajectory, and
	//the function TrajGetState(i) to retrieve a pointer to the i-th state
	std::vector<double> m_trajStates;

	//Overall cost of the solution trajectory (costFreeSpace + costUnknownSpace)
	double m_trajCostOverall;

	//Cost of the solution trajectory corresponding to free space
	double m_trajCostFreeSpace;

	//The remaining path is computed only if the motion planner was not able to compute a solution trajectory to reach the goal.
	//In such a case, the motion planner retrieves the best trajectory that it has found (stored in m_trajStates), and then computes
	//the remaining path to reach the goal.
	//The remaining path does not take dynamics into account. It is just a sequence of (x, y) positions corresponding to cell centers
	std::vector<double> m_remainingPathPoints;

	//Overall cost of the remaining path (costFreeSpace + costUnknownSpace)
	double m_remainingPathCostOverall;

	//Cost of the remaining path corresponding to free space
	double m_remainingPathCostFreeSpace;

	//Goal paramaters
	double m_goalX;
	double m_goalY;
	double m_goalRadius;
	
    };
    
}

#endif

