/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__DECOMPOSITION_HPP_
#define GP__DECOMPOSITION_HPP_

#include "GP/SceneCells.hpp"
#include "GP/Region.hpp"
#include "GP/Goal.hpp"
#include "GP/GoalConnections.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Set.hpp"
#include <vector>

namespace GP
{
    class Decomposition
    {
    public:
	Decomposition() : m_scene(NULL),
			  m_goal(NULL),
			  m_dclearExp(Constants::DECOMPOSITION_EXPONENT_CLEARANCE),
			  m_dclearMax(5.0),
			  m_radixScale(10000000),
			  m_useRadix(false),
			  m_sumPathCosts(0.0)
	{
	}
	
	virtual ~Decomposition()
	{
	    DeleteItems<Region*>(&m_regions);
	}

	virtual void SetupFromParams(Params * const p)
	{
	    m_dclearExp = p->GetValueAsDouble("DecompositionExponentClearance", m_dclearExp);
	}
	

	SceneCells* GetScene(void)
	{
	    return m_scene;
	}
	
	void SetScene(SceneCells *scene)
	{
	    m_scene = scene;
	}
	
	Goal* GetGoal(void)
	{
	    return m_goal;
	}
	
	void SetGoal(Goal *goal)
	{
	    m_goal = goal;
	}
	

	virtual int  GetNrRegions(void)
	{
	    return m_regions.size();
	}
	
	virtual Region* GetRegionByIndex(const int pos)
	{
	    return m_regions[pos];
	}
	
	virtual Region* GetRegionById(const int id)
	{
	    auto pos = m_mapRegionIdsToPos.find(id);
	    if(pos == m_mapRegionIdsToPos.end())
		return NULL;
	    return m_regions[pos->second];
	}

	virtual void AddRegion(Region *r)
	{
	    m_regions.push_back(r);
	    m_mapRegionIdsToPos[r->GetId()] = m_regions.size() - 1;
	}

	virtual int   LocateRegion(const double p[], const bool  allowInsideObstacleCell = false) 	= 0;

	virtual void DrawRegions(void);
	virtual void DrawEdge(const int rid1, const int rid2);
	virtual void DrawEdges(void);

	virtual void ComputeGoalConnections(void) = 0;

	virtual GoalConnections* GetGoalConnections(void) 
	{
	    return &m_goalConnections;
	}

	virtual void PathFromIdsToPoints(const int n, const int rids[], std::vector<double> *pts);

	virtual void Clear(void);

	virtual void UpdatePathsToGoal(void);

	virtual void UpdatePathsToGoalRadix(void);
	

	virtual void AddIdTreeVertex(const int rid, const int vid);
	
	virtual Region* SelectAvailableRegion(void);

	virtual Region* SelectAvailableRegionAtRandom(void);

	virtual Region* SelectAvailableRegionAtRandomBasedOnWeight(void);

	virtual Region* SelectRegionForSampling(void);
	
	
	virtual void PenalizeSelectedRegion(Region *r, const double dsel);
	
	virtual int GetNrAvailable(void)
	{
	    return m_available.size();
	}

	virtual void CreateRegions(void) = 0;
	virtual void ReadyForMotionPlanning(void);
	virtual void GoalClearance(void);
	
	virtual void GetAvailableVids(std::vector<int> & vids);


	virtual void ClearMP(void)
	{
	    m_goalConnections.Clear();
	    m_available.clear();
	    m_selectForSampling.clear();
	    m_sumPathCosts = 0.0;

	    for(auto & reg: m_regions)
		reg->ClearMP();
	    
	}

	virtual void ExportRegions(std::vector<double> &boxes) = 0;
	 
	virtual int FarthestRegionInUnknownSpace(const int rid);

	virtual int FarthestRegionInUnknownSpaceRadix(const int rid);

	long m_radixScale;
	
	double m_dclearExp;
	double m_dclearMax;
	bool m_useRadix;
	
	//protected:
	SceneCells   *m_scene;
	Goal                 *m_goal;
	std::vector<Region*>  m_regions;
	UseMap(int, int)      m_mapRegionIdsToPos;	
	GoalConnections       m_goalConnections;
        UseSet(int)           m_available;

	double m_sumPathCosts;
	std::vector<Region*> m_selectForSampling;
	
    };
}

#endif


