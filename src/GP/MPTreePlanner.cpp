/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/MPTreePlanner.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Timer.hpp"
#include <iostream>

namespace GP
{
    MPTreePlanner::MPTreePlanner(void) : MPPlanner()
    {
	m_dtolSteer   = Constants::PLANNER_TOLERANCE_STEER;	
	m_extendMaxNrSteps  = Constants::PLANNER_EXTEND_MAX_NR_STEPS;
	m_extendMinNrSteps  = Constants::PLANNER_EXTEND_MIN_NR_STEPS;
	m_probSteer   = Constants::PLANNER_PROBABILITY_STEER;
	m_vidSolved   = m_vidBest = Constants::ID_UNDEFINED;
	m_costToGoalBest = INFINITY;
	
}
    
    MPTreePlanner::~MPTreePlanner(void)
    {
	DeleteItems<Vertex*>(&m_vertices);
    }

    void MPTreePlanner::SetupFromParams(Params * const p)
    {
	MPPlanner::SetupFromParams(p);

	m_dtolSteer   = p->GetValueAsDouble("PlannerToleranceSteer", m_dtolSteer);
	m_extendMaxNrSteps  = p->GetValueAsInt("PlannerExtendMaxNrSteps", m_extendMaxNrSteps);
	m_extendMinNrSteps  = p->GetValueAsInt("PlannerExtendMinNrSteps", m_extendMinNrSteps);
	m_probSteer   = p->GetValueAsDouble("PlannerProbabilitySteer", m_probSteer);

    }
    

    void MPTreePlanner::Initialize(void)
    {
	
	if(!(m_sim->IsStateValid()))
	{
	    printf("initial state is in collision\n");
	    exit(0);
	}

	
	auto v   = NewVertex();
	if(AddVertex(v) < 0)
	{
	    printf("initial state could not be added\n");
	    delete v;
	    exit(0);
	}
    }
    
    int MPTreePlanner::AddVertex(Vertex * const v)
    {	
	v->m_state = m_sim->GetStateAllocator()->New();
	m_sim->GetState(v->m_state);
	v->m_rid = m_decomp->LocateRegion(v->m_state);
	if(v->m_rid < 0)
	    return -1;
	
	m_vertices.push_back(v);
	auto vid = m_vertices.size() - 1;
	
	
	m_decomp->AddIdTreeVertex(v->m_rid,  m_vertices.size() - 1);
	
	if(m_decomp->GetGoal()->IsReached(v->m_state))
	    m_vidSolved = vid;

	auto c = m_decomp->GetRegionById(v->m_rid)->GetPathDataToGoal()->m_cost;
	if(c < m_costToGoalBest)
	{
	    m_costToGoalBest = c;
	    m_vidBest = vid;

	    std::cout << "vidBest = " << m_vidBest << " cost = " << c << std::endl;
	    
	}

	return vid;
    }
    
    void MPTreePlanner::GetReversePath(const int vid, std::vector<int> * const rpath) const
    {
	rpath->clear();
	
	int pid = vid;
	while(pid >= 0)
	{
	    rpath->push_back(pid);

	    if(pid >= (int) m_vertices.size())
		printf("path is wrong ... pid = %d nv = %d size=%d\n", pid, (int) m_vertices.size(), (int) rpath->size());
	    
	    pid = m_vertices[pid]->m_parent;


	}
    }

    
    MPTreePlanner::Status MPTreePlanner::ExtendFrom(const int    vid, 
						    const double target[])
    {
	int        parent  = vid;
	int        i       = 0;
	const double d  = m_sim->DistanceStates(m_vertices[vid]->m_state, target);
	
	const int  nrSteps = ((int) (d/m_sim->GetMinDistOneStep())) + RandomUniformInteger(m_extendMinNrSteps, m_extendMaxNrSteps);
	double dtravel = 0.0;
	
	m_sim->SetState(m_vertices[vid]->m_state);

	m_sim->StartSteerToPosition(target);
	
	const int rid = m_vertices[vid]->m_rid;
	
	for(i = 0; i < nrSteps && GetSolved() < 0; ++i)
	{
	    m_sim->SteerToPosition(target);

	    m_sim->SimulateOneStep();
	    if(!m_sim->IsStateValid())
		return EXTEND_COLLISION;
	    
	    Vertex *vnew = NewVertex();
	    vnew->m_parent = parent;
	
	    if(AddVertex(vnew) < 0)
	    {
		delete vnew;
		return EXTEND_COLLISION;
	    }
		    
	    if(m_sim->HasReachedSteerPosition(target, m_dtolSteer))
	    {
		return EXTEND_TARGET;
	    }
	    
	    parent = m_vertices.size() - 1;
	}

	return EXTEND_NORMAL;
    }
    
    void MPTreePlanner::Draw(void) const
    {
	for(int i = m_vertices.size() - 1; i >= 1; --i)
	{
	    GDrawColor(0, 0, 0);
	    GDrawSegment2D(m_vertices[i]->m_state, m_vertices[m_vertices[i]->m_parent]->m_state);
	}
    }
    
       
}


