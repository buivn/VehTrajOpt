/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/MPPlanner.hpp"

namespace GP
{
    void MPPlanner::GetSolution(const int vid,  Solution * const sol) const
    {
	sol->Clear();

	auto sgrid = m_decomp->GetScene()->GetGrid();
	
	auto dimState = sol->m_dimState = m_sim->GetStateAllocator()->GetDim();
	std::vector<int> path;
	int ridLast = -1;

	GetPath(vid, &path);
	
	for(int i = 0; i < path.size();  ++i)
	{
	    auto s = GetState(path[i]);
	    for(int j = 0; j <  dimState; ++j) 
		sol->m_trajStates.push_back(s[j]);	    
	}

	
	auto nrStates = sol->TrajGetNrStates();
	for(int i = 1; i < nrStates; ++i)
	{
	    auto d = Algebra2D::PointDist(sol->TrajGetState(i-1), sol->TrajGetState(i));
	    auto rid = ridLast = m_decomp->LocateRegion(sol->TrajGetState(i));
	    auto cid = sgrid->GetCellId(sol->TrajGetState(i));	    
	    if(m_decomp->GetScene()->m_cidsFree.find(cid) != m_decomp->GetScene()->m_cidsFree.end())
		sol->m_trajCostFreeSpace += d;
	    sol->m_trajCostOverall += d;
	}

	if(nrStates > 0 && m_decomp->GetGoal()->IsReached(sol->TrajGetState(sol->TrajGetNrStates() - 1)) == true)
	    return;
	
	if(nrStates == 0)
	    ridLast = m_decomp->LocateRegion(GetState(vid));
	else 
	    ridLast = m_decomp->LocateRegion(sol->TrajGetState(sol->TrajGetNrStates()-1));

	if(ridLast < 0)
	    return;
	
	
	path  = m_decomp->GetRegionById(ridLast)->GetPathDataToGoal()->m_path;

	for(int i = 0; i < path.size(); ++i)
	{
	    auto c = m_decomp->GetRegionById(path[i])->GetCentroid();
	    sol->m_remainingPathPoints.push_back(c[0]);
	    sol->m_remainingPathPoints.push_back(c[1]);
	}

	auto nrPoints = sol->RemainingPathGetNrPoints();
	for(int i = 1; i < nrPoints; ++i)
	{
	    auto d = Algebra2D::PointDist(sol->RemainingPathGetPoint(i-1), sol->RemainingPathGetPoint(i));
	    auto cid = sgrid->GetCellId(sol->RemainingPathGetPoint(i));	    
	    if(m_decomp->GetScene()->m_cidsFree.find(cid) != m_decomp->GetScene()->m_cidsFree.end())
		sol->m_remainingPathCostFreeSpace += d;
	    sol->m_remainingPathCostOverall += d;
	}
    }
}


