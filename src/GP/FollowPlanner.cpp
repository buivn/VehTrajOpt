/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/FollowPlanner.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"

namespace GP
{
    FollowPlanner::FollowPlanner(void) : MPTreePlanner()
    {
	m_useFollow = true;
	
	m_dsel                              = Constants::PLANNER_SELECTION_PENALTY;
	m_dtolSteer = m_follow.GetReachTolerance();

	m_guideSeparationDistance = 10.0;
	m_guideWidth = 4.0;
	m_guideBorderExtendBefore = 3.0;
	m_guideBorderExtendAfter = 3.0;
	m_minClearAcceptHint = 0.1;
	

	m_maxCountExploreAbandon = 50;
	m_maxCountFollowRetry = 400;
	m_maxCountFollowAbandon = 2000;

	m_runAsMethod = RUN_AS_NORMAL;
	
    }
    
    FollowPlanner::~FollowPlanner(void)
    {
	ClearGroups();
	
    }
    
    void FollowPlanner::SetupFromParams(Params * const p)
    {
	MPTreePlanner::SetupFromParams(p);
	
	m_dsel                       = p->GetValueAsDouble("PlannerSelectionPenalty", m_dsel);
	
	m_follow.SetupFromParams(p);
	m_dtolSteer =  m_follow.GetReachTolerance();
    }


    FollowPlanner::Group* FollowPlanner::SelectGroup(void)
    {
	double wmax = -HUGE_VAL;
	Group *gmax = NULL;
	
	for(auto & it : m_groups)
	    if(it.second->m_weight > wmax)
	    {
		wmax = it.second->m_weight;
		gmax = it.second;
	    }
	return gmax;
    }

    int FollowPlanner::SelectVertex(const Group * g, const double target[])
    {
//	if(RandomUniformReal() < 1 - m_probSelectNearestVertex)
//	    return g->m_vids[RandomUniformInteger(0, g->m_vids.size() - 1)];
	
	double dmin = HUGE_VAL;
	int    imin = -1;
	double d;
	
	for(auto &it : g->m_vids)
	    if((d = Algebra2D::PointDist(target, m_vertices[it]->m_state)) < dmin)
	    {
		dmin = d;
		imin = it;
	    }
	return imin;
	
    }

    int FollowPlanner::InitializeWithSolution(const std::vector<double> & states)
    {
		if(GetNrVertices()  > 0)
			return 0;
			
		auto dimState = m_sim->GetStateAllocator()->GetDim();
		auto n = states.size();	
		int count = 0;
		auto tmp = m_useFollow;
		m_useFollow = false;
		
		for(int i = 0; i < n;  i += dimState, ++count)
		{
			auto s = &states[i];
			m_sim->SetState(s);
			if(m_sim->IsStateValid() == false)
				break;
			auto rid  = m_decomp->LocateRegion(s);
			if(rid < 0)
				break;
			if(m_decomp->GetRegionById(rid)->GetObstacleClearance() < m_minClearAcceptHint)
				break;
			
			auto v = new Vertex();
			v->m_parent = GetNrVertices() - 1;
			AddVertex(v);	    
		}
		m_useFollow = tmp;
		return count;
    }
    

    int FollowPlanner::AddVertex(FollowPlanner::Vertex * const v)
    {
	if(m_runAsMethod == RUN_AS_RRT)
	{
	    v->m_state = m_sim->GetStateAllocator()->New();
	    m_sim->GetState(v->m_state);
	    
	    m_vertices.push_back(v);
	    auto vid = m_vertices.size() - 1;
	    
	    if(m_decomp->GetGoal()->IsReached(v->m_state))
		m_vidSolved = vid;
	    
	    
	    auto c = Algebra2D::PointDist(v->m_state, m_decomp->GetGoal()->GetPolygon()->GetCentroid());
	    if(c < m_costToGoalBest)
	    {
		m_costToGoalBest = c;
		m_vidBest = vid;
	    }
	    
	    
	    return vid;	
 	}
	
	bool doNotAdd = false;
	
	v->m_state = m_sim->GetStateAllocator()->New();
	m_sim->GetState(v->m_state);

	v->m_rid = m_decomp->LocateRegion(v->m_state);
	if(v->m_rid < 0)
	    return -1;
	
	if(m_useFollow)
	{
	    v->m_nextWaypt = v->m_parent >= 0 ? m_vertices[v->m_parent]->m_nextWaypt : 0;
	    if(m_follow.Reached(v->m_nextWaypt, v->m_state))
		++(v->m_nextWaypt);
	    else if(m_follow.IsInside(v->m_nextWaypt, v->m_state) == false)
		return Constants::ID_UNDEFINED;
	    //doNotAdd = true;
	}
	
	m_vertices.push_back(v);
	auto vid = m_vertices.size() - 1;

	if(m_useFollow)
//	if(doNotAdd == false)
	{
	    Group *g;
	    auto it = m_groups.find(v->m_nextWaypt);
	    if(it == m_groups.end())
	    {
		g = new Group();
		g->m_id = v->m_nextWaypt;
		g->m_weight = m_follow.Weight(g->m_id);
		m_groups.insert(std::make_pair(g->m_id, g));
	    }
	    else
		g = it->second;
	    g->m_vids.push_back(vid);
	}
	
	//if(v->m_nextWaypt >= m_follow.GetNrWaypts())
	// m_vidSolved = vid;

//	int prid;
//	v->m_rid = m_decomp->LocateRegion(v->m_state, &prid);
	m_decomp->AddIdTreeVertex(v->m_rid,  m_vertices.size() - 1);
	
	if(m_decomp->GetGoal()->IsReached(v->m_state))
	    m_vidSolved = vid;

	
	auto c = m_decomp->GetRegionById(v->m_rid)->GetPathDataToGoal()->m_cost;
	if(c < m_costToGoalBest)
	{
	    m_costToGoalBest = c;
	    m_vidBest = vid;
	}


	return vid;	
    }

    
    void FollowPlanner::IterateRRT(void)
    {
	double target[2];
	auto grid = m_decomp->GetScene()->GetGrid();
	auto coin = RandomUniformReal();
	
		       
	if(coin < 0.08)
	    m_decomp->GetGoal()->SampleRandomPointInside(target);
	else if(coin < 0.12)
	    m_decomp->GetGoal()->GetRepresentativePoint(target);	
	else
	{
	    target[0] = RandomUniformReal(grid->GetMin()[0], grid->GetMax()[0]);
	    target[1] = RandomUniformReal(grid->GetMin()[1], grid->GetMax()[1]);
	}
	
	double     d    = 0.0;
	double     dmin = HUGE_VAL;
	int        imin = -1;
	int        last = 0;//std::max<int>(m_vertices.size() - 20000, 0);
	
	for(int i = m_vertices.size() - 1; i >= last; --i)
	    if((d =  Algebra2D::PointDist(target, m_vertices[i]->m_state)) < dmin)
	    {
		dmin = d;
		imin = i;
		
		//	if(RandomUniformReal() < 0.000005)
		//   break;
		
	    }

	ExtendFrom(imin, target);
    }

    
    void FollowPlanner::IterateEST(void)
    {
	double target[2];
	
	auto r = m_decomp->SelectAvailableRegionAtRandom();
	auto vid = r->SelectAvailableTreeVertex();

	m_decomp->SelectRegionForSampling()->SamplePointInside(target);

	ExtendFrom(vid, target);
    }

    void FollowPlanner::IterateExplore(void)
    {
	double target[2];
	
	auto r = m_decomp->SelectAvailableRegion();//AtRandomBasedOnWeight();
	auto vid = r->SelectAvailableTreeVertex();

	m_decomp->SelectRegionForSampling()->SamplePointInside(target);

	ExtendFrom(vid, target);
	  
	m_decomp->PenalizeSelectedRegion(r, m_dsel);
    }

    void FollowPlanner::IterateFollow(void)
    {	
	double target[2];

	if(m_groups.size() == 0)
	    SelectFollow();
	
	auto  g = SelectGroup();
	if(g== NULL)
	    return;
	
	m_follow.Sample(g->m_id, target);
	
	auto vid = SelectVertex(g, target);
	
	ExtendFrom(vid, target);
	
	g->m_weight *= m_dsel;
	if(g->m_weight < Constants::EPSILON)
	    for(auto & it : m_groups)
		it.second->m_weight /= m_dsel;
	
	auto r = m_decomp->GetRegionById(m_vertices[vid]->m_rid);
	m_decomp->PenalizeSelectedRegion(r, m_dsel);
    }
    
    void FollowPlanner::Run(const double tmax)
    {
	Timer::Clock clk;
	Timer::Start(&clk);
	
	if(m_vertices.size() == 0)
	{
	    m_useFollow = false;
	    Initialize();
	    m_useFollow = true;	    
	}

	if(m_runAsMethod == RUN_AS_RRT)
	{
	    m_useFollow = false;
	    while(GetSolved() < 0 && Timer::Elapsed(&clk) < tmax)
		IterateRRT();
	    Stats::GetSingleton()->AddValue("TimeRun", Timer::Elapsed(&clk));
	    return;	    
	}

	
	if(m_runAsMethod == RUN_AS_EST)
	{
	    m_useFollow = false;	    
	    while(GetSolved() < 0 && Timer::Elapsed(&clk) < tmax)
		IterateEST();
	    Stats::GetSingleton()->AddValue("TimeRun", Timer::Elapsed(&clk));
	    return;	    
	}
	
	if(m_runAsMethod == RUN_AS_GUST)
	{
	    m_useFollow = false;
	    while(GetSolved() < 0 && Timer::Elapsed(&clk) < tmax)
		IterateExplore();
	    Stats::GetSingleton()->AddValue("TimeRun", Timer::Elapsed(&clk));
	    return;	    
	}
	
	
	
	int countExplore = 0;
	int countFollow = 0;
	int nrGroups = 0;
	
	while(GetSolved() < 0 && Timer::Elapsed(&clk) < tmax)
	{
	    if(m_useFollow == false)
	    {
		countFollow = 0;		
		++countExplore;
		IterateExplore();
		
		if(countExplore >= m_maxCountExploreAbandon)
		{
		    m_useFollow = true;
		    SelectFollow();
		}
	    }
	    else
	    {
		if(countFollow == 0)
		    nrGroups = m_groups.size();
		
		countExplore = 0;
		++countFollow;
		IterateFollow();
		if(nrGroups < m_groups.size())
		    countFollow = 0;
		else if(countFollow >= m_maxCountFollowRetry && RandomUniformReal() < 0.001)
		{
		    SelectFollow();
		    nrGroups = 0;
		}		
		else if(countFollow >= m_maxCountFollowAbandon)
		    m_useFollow = false;
	    }
	}
	
	Stats::GetSingleton()->AddValue("TimeRun", Timer::Elapsed(&clk));

	
    }


    void FollowPlanner::ClearGroups(void)
    {
	for(auto & it : m_groups)
	{
	    delete it.second;
	    it.second = NULL;
	}
	m_groups.clear();
	
    }

    void FollowPlanner::SelectFollow(void)
    {

	int rid = Constants::ID_UNDEFINED;
	double proot[2];
	

	m_follow.Clear();
	ClearGroups();

	Region *r = m_decomp->SelectAvailableRegion();
	rid = r->GetId();
	
	m_decomp->PenalizeSelectedRegion(r, m_dsel);
	
	const int vid = r->SelectAvailableTreeVertex();
	Vertex *v = m_vertices[vid];
	v->m_nextWaypt = 0;
	Group *g = new Group();
	g->m_id = v->m_nextWaypt;
	g->m_weight = m_follow.Weight(g->m_id);
	g->m_vids.push_back(vid);
	m_groups.insert(std::make_pair(g->m_id, g));
	
	proot[0] = v->m_state[0];
	proot[1] = v->m_state[1];
	
	
	
	GraphPathData<int> *data = m_decomp->GetRegionById(rid)->GetPathDataToGoal();
	std::vector<double> pts;
	
	if(data->m_pts.size() == 0)
	    return;

	if(m_guideSeparationDistance > 0)
	    RegularizePointsAlongPath(data->m_pts.size() / 2, &data->m_pts[0], m_guideSeparationDistance, 2, &pts);
	else
	    pts = data->m_pts;
	
	for(int j = 0; j < (int) pts.size(); j += 2)
	    m_follow.AddWaypt(pts[j], pts[j+1], m_guideWidth);

	
	m_follow.ComputeBoundaries(m_guideWidth, m_guideBorderExtendBefore, m_guideBorderExtendAfter, proot );
	
   
    }
    
}


