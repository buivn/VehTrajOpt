/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/Decomposition.hpp"
#include "Utils/Stats.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GraphSearch.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/Radix.hpp"
#include <queue>
#include <algorithm>
#include <iostream>
#include <unordered_map>

namespace GP
{
    void Decomposition::Clear(void)
    {
	m_mapRegionIdsToPos.clear();
	m_goalConnections.Clear();
	DeleteItems<Region*>(&m_regions);
	m_regions.clear();

	m_sumPathCosts = 0.0;
	m_selectForSampling.clear();
	
	
    }
    
    void Decomposition::DrawRegions(void)
    {
	for(int i = GetNrRegions() - 1; i >= 0; --i)
	    GetRegionByIndex(i)->Draw();
    }
    
    void Decomposition::DrawEdge(const int rid1, const int rid2)
    {
	//printf("drawing edge %d %d\n", rid1, rid2);
	
	GDrawSegment2D(GetRegionById(rid1)->GetCentroid(),   GetRegionById(rid2)->GetCentroid());
    }
    
    void Decomposition::DrawEdges(void)
    {
	for(int i = GetNrRegions() - 1; i >= 0; --i)
	{
	    Region    *r      = GetRegionByIndex(i);
	    const int  n      = r->GetNrNeighbors();
	    const int *neighs = r->GetNeighbors();

	    for(int j = 0; j < n; ++j)
	    {
		//printf("edge %d %d (n=%d)\n", r->GetId(), neighs[j], n);
		
		if(neighs[j] >= 0 && r->GetId() < neighs[j])
		    DrawEdge(r->GetId(), neighs[j]);
	    }
	    
	}
    }
    
    void Decomposition::PathFromIdsToPoints(const int n, const int rids[], std::vector<double> *pts)
    {
	pts->clear();
	for(int i = 0; i < n; ++i)
	{
	    const double *c = GetRegionById(rids[i])->GetCentroid();
	    pts->push_back(c[0]);
	    pts->push_back(c[1]);
	}
	
    }

///////////////Graph Search    
    class SearchDecompositionInfo : public GraphSearchInfo<int>
    {
    public:
	SearchDecompositionInfo(void) :
	    GraphSearchInfo<int>(),
	    m_ridToReach(Constants::ID_UNDEFINED)
	{
	}
	
	 ~SearchDecompositionInfo(void)
	{
	}

	void SetDecomposition(Decomposition *decomp)
	{
	    m_decomp = decomp;
	}
	
	void GetOutEdges(const int rid, 
			 std::vector<int> * const edges,
			 std::vector<double> * const costs = NULL) const
	{
	    double dclear, dclearNeigh;
	    
	    int           n;
	    const int     *neighs;
	    const double *costsNeighs;
	    
	    edges->clear();
	    if(costs)
		costs->clear();
	    if(rid < 0)
	    {
		n           = m_decomp->GetGoalConnections()->GetNrNeighbors();
		neighs      = m_decomp->GetGoalConnections()->GetNeighbors();
		costsNeighs = m_decomp->GetGoalConnections()->GetCostsNeighbors();
		dclear = m_decomp->GetGoal()->GetObstacleClearance();
		
	    }
	    else
	    {
		Region *r = m_decomp->GetRegionById(rid);
		dclear = r->GetObstacleClearance();
		
		n           = r->GetNrNeighbors();
		neighs      = r->GetNeighbors();
		costsNeighs = r->GetCostsNeighbors();
	    }

	    for(int i = n - 1; i >= 0; --i)
		if(neighs[i] >= 0)
		{
		    dclearNeigh = m_decomp->GetRegionById(neighs[i])->GetObstacleClearance();
		    dclear  = std::max(Constants::EPSILON, std::min(dclear, dclearNeigh));
		    dclear = std::min(m_decomp->m_dclearMax, dclear);
		    
		    //geman-mclure
		    //dclear = 10 * dclear * dclear / (4 + dclear);
		    
		   edges->push_back(neighs[i]);
		    if(costs)
			costs->push_back(costsNeighs[i] / pow(dclear,  m_decomp->m_dclearExp));
		}
	}
	
	
	bool IsGoal(const int key) const
	{
	    if(m_ridToReach < 0)
		return false;
	    return key == m_ridToReach;	    
	}
	
	double HeuristicCostToGoal(const int u) const
	{
	    if(m_ridToReach < 0)
		return 0;
	    auto p1 = m_decomp->GetRegionById(u)->GetCentroid();
	    auto p2 = m_decomp->GetRegionById(m_ridToReach)->GetCentroid();

	    return Algebra2D::PointDist(p1, p2);
	    
	}

	// protected:
	Decomposition *m_decomp;
	int m_ridToReach;
	

    };

    class SearchDecompositionInfoFarthest : public GraphSearchInfo<int>
    {
    public:
	SearchDecompositionInfoFarthest(void) : GraphSearchInfo<int>()
	{
	}
	
	 ~SearchDecompositionInfoFarthest(void)
	{
	}

	void SetDecomposition(Decomposition *decomp)
	{
	    m_decomp = decomp;
	}
	
	void GetOutEdges(const int rid, 
			 std::vector<int> * const edges,
			 std::vector<double> * const costs = NULL) const
	{
	    edges->clear();
	    if(costs)
		costs->clear();
	    
	    auto r = m_decomp->GetRegionById(rid);
	    auto 	    n           = r->GetNrNeighbors();
	    auto  neighs      = r->GetNeighbors();
	    auto costsNeighs = r->GetCostsNeighbors();

	    //printf("farthest: rid = %d has %d neighbors\n", rid, n);
	    
	    
	    for(int i = n - 1; i >= 0; --i)
	    {
		edges->push_back(neighs[i]);
		if(costs)
		    costs->push_back(costsNeighs[i]);
	    }
	}
	
	
	bool IsGoal(const int key) const
	{
	    return false;
	}
	
	double HeuristicCostToGoal(const int u) const
	{
	    return 0;
	}

    protected:
	Decomposition *m_decomp;
	

    };

    int Decomposition::FarthestRegionInUnknownSpace(const int ridStart)
    {
	if(m_useRadix)
	    return FarthestRegionInUnknownSpaceRadix(ridStart);
	
	double c[2];
	
	SearchDecompositionInfoFarthest info;
	GraphSearch<int>        search;
	int                     goal;
	int ridWithMaxCostNoClearance = -1;
	double maxCostNoClearance = -1;
	
	int ridWithMaxCostClearance1 = -1;
	double maxCostClearance1 = -1;
	
	int ridWithMaxCostClearance2 = -1;
	double maxCostClearance2 = -1;
	
	info.SetDecomposition(this);
	search.SetInfo(&info);
	search.AStar(ridStart, false, &goal);

//	printf("cidsUnknown = %d\n", m_scene->m_cidsUnknown.size());
	
	for(auto & cid : m_scene->m_cidsUnknown)
	{
	    m_scene->GetGrid()->GetCellCenterFromId(cid, c);
	    
	    auto rid = LocateRegion(c);

	    //    printf("cid = %d rid = %d valid = %d\n", cid, rid, GetRegionById(rid)->IsValid());
	    
	    
	    if(rid < 0  || GetRegionById(rid)->IsValid() == false)
		continue;
	    
	    auto cost = search.GetPathCostFromStart(rid);
	    auto dclear =GetRegionById(rid)->GetObstacleClearance();

	    //printf("considering region %d cost = %f dclear = %f maxCost = %f\n", rid, cost, dclear, maxCostNoClearance);

	    if(cost != HUGE_VAL && cost > maxCostClearance2 && dclear > 2.0)
	    {
		ridWithMaxCostClearance2 = rid;
		maxCostClearance2 = cost;
	    }
	    	
	    if(cost != HUGE_VAL && cost > maxCostClearance1 && dclear > 1.0)
	    {
		ridWithMaxCostClearance1 = rid;
		maxCostClearance1 = cost;
	    }

	    if(cost != HUGE_VAL && cost > maxCostNoClearance)
	    {
		ridWithMaxCostNoClearance = rid;
		maxCostNoClearance = cost;
	    }
	}

	if(ridWithMaxCostClearance2 >= 0)
	    return ridWithMaxCostClearance2;
	if(ridWithMaxCostClearance1 >= 0)
	    return ridWithMaxCostClearance1;
	
	return ridWithMaxCostNoClearance;
	
    }

    
    int Decomposition::FarthestRegionInUnknownSpaceRadix(const int ridStart)
    {
	double c[2];
	
	int ridWithMaxCostNoClearance = -1;
	double maxCostNoClearance = -1;
	
	int ridWithMaxCostClearance1 = -1;
	double maxCostClearance1 = -1;
	
	int ridWithMaxCostClearance2 = -1;
	double maxCostClearance2 = -1;
std::unordered_map<int, int> mapRidsToRadixIndices;

	
	RadixGraph radix;
	
	//add nodes
	for(int i = 0; i < GetNrRegions() ; ++i)
	    if(GetRegionByIndex(i)->IsValid())
	    {
		auto rid = GetRegionByIndex(i)->GetId();
		mapRidsToRadixIndices.insert(std::make_pair(rid, radix.nodes.size()));
		radix.nodes.push_back(new Node(rid));
	    }
	auto curr = mapRidsToRadixIndices.find(ridStart);
	if(curr == mapRidsToRadixIndices.end())
	    return -1;
	auto indexStart = curr->second;
	
	
	//add  edges
	for(int i = 0; i < radix.nodes.size(); ++i)
	{
	    auto r = GetRegionById(radix.nodes[i]->m_id);
	    auto dclear = r->GetObstacleClearance();		
	    auto nrNeighs = r->GetNrNeighbors();
	    auto neighs = r->GetNeighbors();
	    auto costs = r->GetCostsNeighbors();
	    
	    for(int k = 0; k < nrNeighs; ++k)
	    {
		auto neigh = neighs[k];
		int j = mapRidsToRadixIndices.find(neigh)->second;		
		if(j >= 0)
		{
		    auto dclearNeigh = GetRegionById(neigh)->GetObstacleClearance();
		    auto dclearDist  = std::min(m_dclearMax, std::max(Constants::EPSILON, std::min(dclear, dclearNeigh)));
		    auto cost = costs[k];// / pow(dclearDist, m_dclearExp);

		    auto relVal = m_radixScale * cost;
		    RadixValType val;
		    
		    if(relVal > ULLONG_MAX)
			val = (RadixValType) (LLONG_MAX);
		    else
			val = (RadixValType) relVal;
		    
		    radix.addEdge(i, j, val);
		}
	    }	    
	}

	//compute paths
	
	radix.dijkstra(radix.nodes[indexStart]);


	for(auto & cid : m_scene->m_cidsUnknown)
	{
	    m_scene->GetGrid()->GetCellCenterFromId(cid, c);
	    
	    auto rid = LocateRegion(c);
	    if(rid < 0)
		continue;
	    auto curr = mapRidsToRadixIndices.find(rid);
	    if(curr == mapRidsToRadixIndices.end())
		continue;	    
	    auto indexRadix = curr->second;
	    
	    auto cost = radix.pathCost(radix.nodes[indexRadix]);
	    auto dclear =GetRegionById(rid)->GetObstacleClearance();

	    
	    if(cost != HUGE_VAL && cost > maxCostClearance2 && dclear > 2.0)
	    {
		ridWithMaxCostClearance2 = rid;
		maxCostClearance2 = cost;
	    }
	    	
	    if(cost != HUGE_VAL && cost > maxCostClearance1 && dclear > 1.0)
	    {
		ridWithMaxCostClearance1 = rid;
		maxCostClearance1 = cost;
	    }
	    	
	    if(cost != HUGE_VAL && cost > maxCostNoClearance)
	    {
		ridWithMaxCostNoClearance = rid;
		maxCostNoClearance = cost;
	    }
	}

	if(ridWithMaxCostClearance2 >= 0)
	    return ridWithMaxCostClearance2;
	if(ridWithMaxCostClearance1 >= 0)
	    return ridWithMaxCostClearance1;
	return ridWithMaxCostNoClearance;
	
    }
    
	
	
    void Decomposition::UpdatePathsToGoal(void)
    {
	m_sumPathCosts = 0.0;
	m_selectForSampling.clear();
	
	if(m_useRadix)
	    return UpdatePathsToGoalRadix();
	
	Timer::Clock clk;
	Timer::Start(&clk);

	SearchDecompositionInfo info;
	GraphSearch<int>        search;
	int                     goal;
	GraphPathData<int>     *data;
	Region                 *r;
	double                  p[2];
	const int               n = GetNrRegions();

	
	info.SetDecomposition(this);
	search.SetInfo(&info);
	search.AStar(-1, false, &goal);

	for(int i = 0; i < n; ++i)
	{
	    r = GetRegionByIndex(i);
	    r->m_dijkstraParent = search.GetParent(r->GetId());
	    
	    data = r->GetPathDataToGoal();
	    data->m_cost = search.GetPathCostFromStart(r->GetId());
	    search.GetReversePathFromStart(r->GetId(), &data->m_path);
	    data->m_hasPath = data->m_path.size() > 0;
	    if(data->m_path.size() > 0)
	    {
		data->m_path.pop_back(); //remove -1 (goal id)

		PathFromIdsToPoints(data->m_path.size(), &data->m_path[0], &data->m_pts);
		data->m_pts.push_back(GetGoal()->GetPolygon()->GetCentroid()[0]);
		data->m_pts.push_back(GetGoal()->GetPolygon()->GetCentroid()[1]);

		r->SetWeightBasedOnPathCost();
		
		m_sumPathCosts += data->m_cost;
		m_selectForSampling.push_back(r);
		
	    }
	}

	 
	Stats::GetSingleton()->AddValue("TimeUpdatePathsToGoal", Timer::Elapsed(&clk));
	
    }

    
    void Decomposition::UpdatePathsToGoalRadix(void)
    {
	
	m_sumPathCosts = 0.0;
	m_selectForSampling.clear();
	
	Timer::Clock clk;
	Timer::Start(&clk);

//	double minEdgeCost = INFINITY;
//	double maxEdgeCost = 0;
//	double minScale = INFINITY;
//	double maxScale = 0;
	std::unordered_map<int, int> mapRidsToRadixIndices;

	auto gid = GetNrRegions();
	
	RadixGraph radix;
	int count = 0;
	int nrNeighs;
	const int *neighs;
	const double *costs;
	double dclear;
	
	//add nodes
	for(int i = GetNrRegions() - 1; i >= 0; --i)
	    if(GetRegionByIndex(i)->IsValid())
	    {
		auto rid = GetRegionByIndex(i)->GetId();
		mapRidsToRadixIndices.insert(std::make_pair(rid, radix.nodes.size()));
		radix.nodes.push_back(new Node(rid));
	    }
	mapRidsToRadixIndices.insert(std::make_pair(gid, radix.nodes.size()));	
	radix.nodes.push_back(new Node(gid));
	
	//add  edges
	for(int i = 0; i < radix.nodes.size(); ++i)
	{
	    if(i == radix.nodes.size() - 1) //last is goal
	    {
		nrNeighs  = GetGoalConnections()->GetNrNeighbors();
		neighs      = GetGoalConnections()->GetNeighbors();
		costs = GetGoalConnections()->GetCostsNeighbors();
		dclear = m_goal->GetObstacleClearance();		
	    }
	    else
	    {
		auto r = GetRegionById(radix.nodes[i]->m_id);
		dclear = r->GetObstacleClearance();		
		nrNeighs = r->GetNrNeighbors();
		neighs = r->GetNeighbors();
		costs = r->GetCostsNeighbors();
	    }
	    
	    for(int k = 0; k < nrNeighs; ++k)
	    {
		auto neigh = neighs[k];
		int j = mapRidsToRadixIndices.find(neigh)->second;		
		if(j >= 0)
		{
		    auto dclearNeigh = GetRegionById(neigh)->GetObstacleClearance();
		    auto dclearDist  = std::min(m_dclearMax, std::max(0.001, std::min(dclear, dclearNeigh)));		    
		    auto cost = costs[k] / pow(dclearDist, m_dclearExp);
/*
		    auto suse = 1.0 / pow(dclearDist, m_dclearExp);		    
		    if(suse < minScale)
			minScale = suse;
		    if(suse > maxScale)
			maxScale= suse;
		    
		    if(cost < minEdgeCost)
			minEdgeCost = cost;
		    if(cost > maxEdgeCost)
			maxEdgeCost = cost;
*/

		    auto relVal = m_radixScale * cost;
		    RadixValType val;
		    
		    if(relVal > ULLONG_MAX)
			val = (RadixValType) (LLONG_MAX);
		    else
			val = (RadixValType) relVal;
		    
		    radix.addEdge(i, j,  val);
		}
	    }	    
	}

//	printf("min/max edge costs %12.10f %12.10f = %12.10f %12.10f\n", minEdgeCost, maxEdgeCost, minScale, maxScale);
	

	//compute paths
	radix.dijkstra(radix.nodes.back());

	
	double                  p[2];
	const int         n = GetNrRegions();
	
	
	for(int i = 0; i < radix.nodes.size() - 1; ++i) //except the goal
	{
	    auto r = GetRegionById(radix.nodes[i]->m_id);
	    auto data = r->GetPathDataToGoal();
	    data->m_cost =((double) (radix.path(radix.nodes[i], data->m_path))) / m_radixScale;
	    data->m_hasPath = data->m_path.size() > 0;
	    
	    if(data->m_path.size() > 0)
	    {
		data->m_path.pop_back(); //remove  last one (goal id)

		PathFromIdsToPoints(data->m_path.size(), &data->m_path[0], &data->m_pts);
		GetGoal()->GetRepresentativePoint(p);
		data->m_pts.push_back(p[0]);
		data->m_pts.push_back(p[1]);
/*
		printf("data->m_cost = %f [part of goal connections %d]\n", data->m_cost, m_goalConnections.FindNeighbor(r->GetId()));
		printf("path = <");
		for(int k = 0; k < data->m_path.size(); ++k)
		    printf("%d ", data->m_path[k]);
		printf(">\n");
*/
		
		r->SetWeightBasedOnPathCost();

		
		m_sumPathCosts += data->m_cost;
		m_selectForSampling.push_back(r);
		
	    }
	    
	}

	
	Stats::GetSingleton()->AddValue("TimeUpdatePathsToGoal", Timer::Elapsed(&clk));
	
    }
    
    void Decomposition::AddIdTreeVertex(const int rid, const int vid)
    {
	if(rid < 0)
	    return;
	
	Region *r = GetRegionById(rid);
	if(r->GetNrTreeVertices() == 0)
	    m_available.insert(rid);
	r->AddIdTreeVertex(vid);
    }

    void Decomposition::GetAvailableVids(std::vector<int> & vids)
    {
	vids.clear();
	for(auto &rid : m_available)
	{
	    auto r = GetRegionById(rid);
	    vids.push_back(r->m_vids[RandomUniformInteger(0, r->m_vids.size() - 1)]);
	}	
    }
    
    Region* Decomposition::SelectAvailableRegion(void)
    {
	Timer::Clock clk;
	Timer::Start(&clk);
	
	Region *r;
	Region *rsel = NULL;
	double  wmax = -HUGE_VAL;
	double w;
	
//	std::cout << "available regions " << m_available.size() << std::endl;

	for(auto &rid : m_available)
	{
	    r = GetRegionById(rid);
	    auto data = r->GetPathDataToGoal();
	    if(data->m_path.size() == 0 || data->m_cost == INFINITY)
		continue;
	    
	    auto n = r->m_vids.size();

	    w = r->GetWeight();
	    
	    if(w > wmax)
	    {
		rsel = r;
		wmax = w;
	    }
	}
	
	return rsel;
	
    }

    Region* Decomposition::SelectRegionForSampling(void)
    {
        double coin = RandomUniformReal(0, m_sumPathCosts);
	double w = 0.0;

	for(auto & r : m_selectForSampling)
	{
	    w += r->GetPathDataToGoal()->m_cost;
	    if(w + Constants::EPSILON >= coin)
		return r;
	}

	return m_selectForSampling[RandomUniformInteger(0, m_selectForSampling.size() - 1)];
	
	
    }


    Region* Decomposition::SelectAvailableRegionAtRandomBasedOnWeight(void)
    {
	double w = 0.0;
	for(auto & rid : m_available)
	    w += GetRegionById(rid)->GetWeight();
	double coin = RandomUniformReal(0, w);

	//printf("available regions %d with weight %f coin %f\n", m_available.size(), w, coin);
	//for(auto &rid : m_available)
	//printf("  region %d has weight %f\n", rid, GetRegionById(rid)->GetWeight());
	
	
	w = 0.0;
	
	for(auto &rid : m_available)
	{
	    w += GetRegionById(rid)->GetWeight();
	    if(w + Constants::EPSILON >= coin)
		return GetRegionById(rid);
	}
	
	return NULL;
	
    }

    Region* Decomposition::SelectAvailableRegionAtRandom(void)
    {
	int pos = RandomUniformInteger(0, m_available.size() - 1);
	
	for(auto &rid : m_available)
	    if((--pos) < 0)
		return GetRegionById(rid);
	return NULL;
	
    }

    void Decomposition::PenalizeSelectedRegion(Region *r, const double dsel)
    {
	double w = r->GetWeight() * dsel;
	
	r->SetWeight(w);
	if(w <= Constants::EPSILON)
	    for(auto &rid : m_available)
	    {
		Region *rother = GetRegionById(rid);
		rother->SetWeight(rother->GetWeight() / dsel);
	    }
    }
    

    void Decomposition::ReadyForMotionPlanning(void)
    {
	ComputeGoalConnections();	
	GoalClearance();	
	UpdatePathsToGoal();
    }

    void Decomposition::GoalClearance(void)
    {
	const bool allowInsideObstacleCell = true;
	
	
	double p[2];
	m_goal->GetRepresentativePoint(p);
	const int rid = LocateRegion(p, allowInsideObstacleCell);
	if(rid >= 0)
	    m_goal->SetObstacleClearance(GetRegionById(rid)->m_dclear);
	else
	    m_goal->SetObstacleClearance(Constants::EPSILON);
	
    }
    
    
    
    
}

