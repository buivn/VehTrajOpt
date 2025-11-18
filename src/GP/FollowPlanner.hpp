/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__FOLLOW_PLANNER_HPP_
#define GP__FOLLOW_PLANNER_HPP_

#include "GP/MPTreePlanner.hpp"
#include "GP/Follow.hpp"
#include "GP/MPSimulator.hpp"
#include "GP/Decomposition.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Map.hpp"
#include <vector>

namespace GP
{
    class FollowPlanner : public MPTreePlanner
    {
    public:
	FollowPlanner(void);
	
	virtual ~FollowPlanner(void);

	virtual void SetupFromParams(Params * const p);

	virtual int InitializeWithSolution(const std::vector<double> & states);
	
	virtual void Run(const double tmax);
 	
	Follow                m_follow;
		
    protected:
	struct Group
	{
	    Group(void) :
		m_id(Constants::ID_UNDEFINED),
		m_weight(0.0)
	    {
	    }
	    
	    int              m_id;
	    double           m_weight;
	    std::vector<int> m_vids;
	};
		    

	
	virtual int AddVertex(Vertex * const v);

	virtual Group* SelectGroup(void);

	virtual int SelectVertex(const Group * g, const double target[]);

	virtual void SelectFollow(void);

	virtual void ClearGroups(void);
		
	    
	UseMap(int, Group*)   m_groups;
	
	double m_dsel;
	
    public:
	double m_guideSeparationDistance;
	double  m_guideWidth;
	double m_guideBorderExtendBefore ;
	double m_guideBorderExtendAfter;
	double m_minClearAcceptHint;
	

	int m_maxCountExploreAbandon;
	int m_maxCountFollowAbandon;	
	int m_maxCountFollowRetry;
	
	
    public:
	enum
	    {
		RUN_AS_NORMAL = 0,
		RUN_AS_RRT = 1,
		RUN_AS_GUST = 2,
		RUN_AS_EST = 3
	    };
	
	    
	bool m_useFollow;
	int m_runAsMethod;
	

	virtual void IterateRRT(void);	
	virtual void IterateEST(void);	
	virtual void IterateExplore(void);
	virtual void IterateFollow(void);
	
	virtual void ResetForNewMP(void)
	{
	    MPTreePlanner::ResetForNewMP();
	    ClearGroups();
	    m_follow.Clear();
	    m_useFollow = true;
	    
	}
	
    };    
}

#endif

