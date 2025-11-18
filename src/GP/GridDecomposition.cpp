/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/GridDecomposition.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Utils/Set.hpp"
#include <cmath>
#include <queue>
#include <unordered_set>


namespace GP
{
    void GridDecomposition::ExportRegions(std::vector<double> &boxes)
    {
	boxes.clear();	
	for(int i = 0; i < m_regions.size(); ++i)
	{
	    auto r = dynamic_cast<GridRegion*>(m_regions[i]);
	    boxes.push_back(r->m_bbox[0]);
	    boxes.push_back(r->m_bbox[1]);
	    boxes.push_back(r->m_bbox[2]);
	    boxes.push_back(r->m_bbox[3]);
	}
	
    }
    

    void GridDecomposition::CreateRegions(void)
    {
	Timer::Clock clk;
	Timer::Start(&clk);
	
	std::unordered_set<int> ridsOccupied;	
	std::queue<GridRegion*> occupied;
	std::vector<double> p(GetScene()->GetGrid()->GetNrDims());
	double bbox[4];
	
	std::queue<GridRegion*> subdiv;
	std::vector<GridRegion*> unused;	
	GridRegion *rsub1;
	GridRegion *rsub2;
	int nextId = 0;
	
	auto minArea = GetScene()->GetGrid()->GetCellVolume();
	
	
	for(int i = 0; i < m_grid.GetNrCells(); ++i)
	{
	    auto r = new GridRegion();
	    r->SetDecomposition(this);
	    r->SetId(i);
	    m_grid.GetCellFromId(i, r->m_bbox);
	    r->m_area = m_grid.GetCellVolume();
	    r->UpdateCentroid();
	    AddRegion(r);
	}
	nextId = m_grid.GetNrCells();
	
	for(auto & cid : GetScene()->m_cidsObstacles)
	{
	    GetScene()->GetGrid()->GetCellCenterFromId(cid, &p[0]);
	    GetScene()->GetGrid()->GetCellFromId(cid, &bbox[0]);
	    
	    auto rid = m_grid.GetCellId(&p[0]);
	    auto r = dynamic_cast<GridRegion*>(GetRegionById(rid));
	    r->m_cidsObstacles.push_back(cid);
	    r->m_cellCentersObstacles.push_back(p[0]);
	    r->m_cellCentersObstacles.push_back(p[1]);	    
	    r->m_cellBoundsObstacles.push_back(bbox[0]);
	    r->m_cellBoundsObstacles.push_back(bbox[1]);	    
	    r->m_cellBoundsObstacles.push_back(bbox[2]);
	    r->m_cellBoundsObstacles.push_back(bbox[3]);

	    ridsOccupied.insert(rid);
	}

	for(auto & rid : ridsOccupied)
	    occupied.push(dynamic_cast<GridRegion*>(GetRegionById(rid)));

	while(!occupied.empty())
	{
	    auto r = occupied.front();
	    occupied.pop();

	    subdiv.push(r);	    
	    while(!subdiv.empty())
	    {
		auto rchild = subdiv.front();
		subdiv.pop();
		if(rchild->GetArea() <= minArea)
		    continue;

		if(rchild != r)
		    unused.push_back(rchild);
		
		rsub1 = new GridRegion();
		rsub2 = new GridRegion();
		rchild->Split(*rsub1, *rsub2);
		if(rsub1->GetArea() > minArea && rsub1->m_cidsObstacles.size() > 0)
		    subdiv.push(rsub1);
		else
		    r->m_children.push_back(rsub1);		
		if(rsub2->GetArea() > minArea && rsub2->m_cidsObstacles.size() > 0)
		    subdiv.push(rsub2);
		else
		    r->m_children.push_back(rsub2);
	    }
	    DeleteItems<GridRegion*>(&unused);
	    unused.clear();

	    double tarea = 0.0;
	    
	    for(int i  = 0; i < r->m_children.size(); ++i)
	    {
		auto rchild = r->m_children[i];
		rchild->m_parent = r;
		rchild->SetId(nextId);
		tarea += rchild->m_area;		
		++nextId;
		AddRegion(rchild);
		//	printf("child %d/%d of %d has area %f minArea = %f parent= %f overall=%f\n", i, r->m_children.size(), r->GetId(), rchild->m_area, minArea, r->m_area, tarea);
		
	    }

	    // printf("region %d has %d children minArea = %f\n", r->GetId(), r->m_children.size(), minArea);
	    
	}

	Stats::GetSingleton()->AddValue("TimeDecompRegions", Timer::Elapsed(&clk));

	Timer::Start(&clk);
	
	
	//add neighbors
	auto nrCells = m_grid.GetNrCells();
	std::vector<int> cidsNeighs;
	std::vector<GridRegion*> process1;
	std::vector<GridRegion*> process2;

	int nrEdges = 0;
	
	
	
	for(int i = 0; i < nrCells; ++i)
	{
	    process1.clear();
	    process2.clear();
	    
	    auto r = dynamic_cast<GridRegion*>(GetRegionById(i));
	    
	    if(r->m_children.empty())
		process1.push_back(r);
	    else
	    {
		for(auto & rchild : r->m_children)
		    if(rchild->m_cidsObstacles.empty())
		    {
			process1.push_back(rchild);
			process2.push_back(rchild);
		    }
	    }
	    
	    m_grid.GetNeighs(r->GetId(), &cidsNeighs);
	    for(auto & cidNeigh : cidsNeighs)
	    {
		auto rneigh = dynamic_cast<GridRegion*>(GetRegionById(cidNeigh));
		if(rneigh->m_children.empty())
		    process2.push_back(rneigh);
		else
		{
		    for(auto & rchild : rneigh->m_children)
			//if(rchild->m_cidsObstacles.empty())
			    process2.push_back(rchild);
		}
	    }
	    
	    for(int j =  0; j < process1.size(); ++j)
	    {
		auto rcurr1 = process1[j];
		for(int k = 0; k < process2.size(); ++k)
		{
		    auto rcurr2 = process2[k];
		    if(rcurr1 != rcurr2 && rcurr1->IsNeighbor(rcurr2->m_bbox))
		    {
			auto d = Algebra::PointDist(m_grid.GetNrDims(), rcurr1->GetCentroid(), rcurr2->GetCentroid());
	
			if(rcurr1->m_cidsObstacles.empty() && rcurr2->m_cidsObstacles.empty())
			{
			    rcurr1->AddNeighbor(rcurr2->GetId(), d);
			    rcurr2->AddNeighbor(rcurr1->GetId(), d);
			}
			else if(rcurr2->m_cidsObstacles.empty()  == false)
			{
			    if(rcurr1->m_dclear > d)
				rcurr1->m_dclear = d;			    
			}
			else if(rcurr1->m_cidsObstacles.empty()  == false)
			{
			    if(rcurr2->m_dclear > d)
				rcurr2->m_dclear = d;			    
			}
			
			
			
			//		printf("adding edge %d %d\n", rcurr1->GetId(), rcurr2->GetId());

			++nrEdges;			
		    }
		}
	    }
	}

	Stats::GetSingleton()->AddValue("TimeDecompEdges", Timer::Elapsed(&clk));
	Timer::Start(&clk);
	

	//printf("nrRegions = %d nrEdges = %d nrCells = %d nrSceneCells = %d\n",
	//   GetNrRegions(), nrEdges, m_grid.GetNrCells(), GetScene()->m_grid.GetNrCells());

	//compute clearances
	auto cmp = [](GridRegion* left, GridRegion* right) { return left->m_dclear > right->m_dclear; };
	std::priority_queue<GridRegion*, std::vector<GridRegion*>, decltype(cmp)> qclear(cmp);
	double dclear;
	
	for(int i = GetNrRegions() - 1; i >= 0; --i)
	{
	    auto r = dynamic_cast<GridRegion*>(GetRegionByIndex(i));
	    if(r->m_children.empty() == false)
		continue;
	    if(r->m_cidsObstacles.empty() == false)
	    {
		r->m_dclear = 0.0;
		//qclear.push(r);
	    }
	    else if(r->IsBoundary(m_grid.GetMin(), m_grid.GetMax(), dclear))
	    {
		if(r->m_dclear > dclear)
		    r->m_dclear = dclear;
	  	qclear.push(r);
	    }
	    else if(r->m_dclear != INFINITY)
		qclear.push(r);	    
	}

	while(qclear.empty() == false)
	{
	    auto r = qclear.top();
	    qclear.pop();

	    //printf("clearance..processing %d with clearance %f\n", r->GetId(), r->m_dclear);
	    
	    auto neighs  = r->GetNeighbors();
	    auto costs = r->GetCostsNeighbors();
	    
	    for(int i = r->GetNrNeighbors() - 1; i >= 0; --i)
	    {
		auto rneigh = dynamic_cast<GridRegion*>(GetRegionById(neighs[i]));
		auto dclear =  r->m_dclear + costs[i];
		if(rneigh->m_dclear > dclear)
		{
		    rneigh->m_dclear = dclear;
		    qclear.push(rneigh);
		}
	    }
	}

	Stats::GetSingleton()->AddValue("TimeDecompClearances", Timer::Elapsed(&clk));
	
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	GridRegion *r;
	Region     *rneigh;
	std::vector<int> neighs;
	std::vector<double> p(GetScene()->GetGrid()->GetNrDims());	
	std::queue<int> q;
	
	for(int i = 0; i < m_grid.GetNrCells(); ++i)
	{
	    r = new GridRegion();
	    r->SetDecomposition(this);
	    r->SetId(i);
	    //  r->SetCenter(i);
	    m_grid.GetCellFromId(i, r->m_bbox);
	    r->m_area = m_grid.GetCellVolume();
	    r->UpdateCentroid();
	    AddRegion(r);

	    auto cid = GetScene()->GetGrid()->GetCellId(r->GetCentroid());
	    
	    if(GetScene()->m_cidsObstacles.find(cid) != GetScene()->m_cidsObstacles.end())
	    {
		r->m_dclear = 0;
		q.push(r->GetId());
	    }
	    else if(m_grid.IsBorderCell(i))
		r->m_dclear = 1.0;
	    else
		r->m_dclear = INFINITY;
	}

	for(auto & cid : GetScene()->m_cidsObstacles)
	{
	    GetScene()->GetGrid()->GetCellCenterFromId(cid, &p[0]);
	    auto rid = m_grid.GetCellId(&p[0]);
	    r = dynamic_cast<GridRegion*>(GetRegionById(rid));
	    r->m_dclear = 0;
	    q.push(rid);
	}

	for(int i = 0; i < m_grid.GetNrCells(); ++i)
	{
	    r = dynamic_cast<GridRegion*>(GetRegionById(i));
	    if(r->m_dclear <= Constants::EPSILON)
		continue;
	    else if (r->m_dclear < INFINITY)
		q.push(r->GetId());
	    
	    m_grid.GetNeighs(i, &neighs);
	    for(auto & neigh : neighs)
	    {
		rneigh = GetRegionById(neigh);
		if(i < neigh && rneigh->m_dclear > Constants::EPSILON)
		{
		    auto d = Algebra::PointDist(m_grid.GetNrDims(), r->GetCentroid(), rneigh->GetCentroid());
		    r->AddNeighbor(neigh, d);
		    rneigh->AddNeighbor(i, d);
		}
	    }
	}

	//compute clearances
	while(q.empty() == false)
	{
	    auto rid = q.front();
	    q.pop();

	    r =  dynamic_cast<GridRegion*>(GetRegionById(rid));
	    
    	    m_grid.GetNeighs(rid, &neighs);
	    for(auto & neigh : neighs)
	    {
		rneigh = GetRegionById(neigh);
		if(rneigh->m_dclear == INFINITY)
		{
		    rneigh->m_dclear = r->m_dclear + 1.0;
		    q.push(neigh);
		}
	    }
	}
	*/
	
    }
    
    int GridDecomposition::LocateRegion(const double p[], const bool  allowInsideObstacleCell)
    {
	const int rid = m_grid.GetCellId(p);
	
	auto r = dynamic_cast<GridRegion*>(GetRegionById(rid));

	if(r == NULL)
	    return Constants::ID_UNDEFINED;
	
	if(r->m_children.empty())
	{
	    if(allowInsideObstacleCell)
		return rid;	    
	    return r->m_cidsObstacles.empty() ? rid : Constants::ID_UNDEFINED;
	}
	
	for(auto & rchild : r->m_children)
	    if(rchild->IsPointInside(p))
	    {
		if(allowInsideObstacleCell)
		    return rchild->GetId();
		return rchild->m_cidsObstacles.empty() ? rchild->GetId() : Constants::ID_UNDEFINED;
	    }
	
	return Constants::ID_UNDEFINED;
    }
 

    void GridDecomposition::ComputeGoalConnections(void)
    {
	double p[2];
	double c[2];
	std::vector<int> cidsInside;
	std::vector<int> cidsIntersect;
	std::unordered_set<int> rids;
	
	
	GetGoal()->GetRepresentativePoint(p);
	m_goalConnections.Clear();

	auto poly = GetGoal()->GetPolygon();
	poly->OccupiedGridCells(GetScene()->GetGrid(), &cidsInside, &cidsIntersect);
	cidsInside.insert(cidsInside.end(), cidsIntersect.begin(), cidsIntersect.end());	
	for(auto & cid : cidsInside)
	{
	    GetScene()->GetGrid()->GetCellCenterFromId(cid, p);
	    auto rid = LocateRegion(p);
	    if(rid >= 0)
		rids.insert(rid);
	}
	for(auto & rid : rids)
	{
	    auto r = GetRegionById(rid);
	    m_goalConnections.AddNeighbor(rid, Algebra2D::PointDist(r->GetCentroid(), poly->GetCentroid()));
	}
    }
    
    
}

