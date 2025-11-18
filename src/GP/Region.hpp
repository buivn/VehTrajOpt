/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__REGION_HPP_
#define GP__REGION_HPP_

#include "Utils/GraphSearch.hpp"
#include "Utils/Constants.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Map.hpp"
#include "Utils/Polygon2D.hpp"
#include <vector>

namespace GP
{
    class Decomposition;
    
    class Region
    {
    public:
	friend class Decomposition;
	friend class PrimitiveDecomposition;

	int m_dijkstraParent;
	
	Region(void) : m_decomp(NULL),
		       m_rid(Constants::ID_UNDEFINED),
		       // m_center(Constants::ID_UNDEFINED),
		       m_weight(0.0),
		       m_nrSels(0),
		       m_area(0.0),
		       m_dclear(INFINITY)
	{
	    m_centroid[0] = m_centroid[1] = m_centroid[2] = 0.0;
	    m_dijkstraParent = -1;
	}

	virtual ~Region(void)
	{
	}
		
	virtual void Clear(void)
	{
	    m_weight = 0.0;
	    m_nrSels = 0.0;
	    m_vids.clear();
	    m_neighs.clear();
	    m_costs.clear();
	    m_pathDataToGoal.m_cost = 0.0;
	    m_pathDataToGoal.m_path.clear();
	    m_pathDataToGoal.m_pts.clear();
	}
	
	virtual Decomposition* GetDecomposition(void)
	{
	    return m_decomp;
	}
	
	virtual int GetId(void) 
	{
	    return m_rid;
	}
	
	/*virtual int GetCenter(void)
	{
	    return m_center;
	    }*/
	
	virtual const double* GetCentroid(void) 
	{
	    return m_centroid;
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

	virtual double GetArea(void)
	{
	    return m_area;
	}
	
	
	virtual int GetNrTreeVertices(void)
	{
	    return m_vids.size();
	}
	
	virtual const int* GetIdsTreeVertices(void) 
	{
	    return &m_vids[0];
	}

	virtual int GetNrTimesSelected(void) 
	{
	    return m_nrSels;
	}
	
	virtual double GetWeight(void) 
	{
	    return m_weight;
	}

	
	virtual GraphPathData<int>* GetPathDataToGoal(void)
	{
	    return &m_pathDataToGoal;
	}

	virtual void SetDecomposition(Decomposition *decomp)
	{
	    m_decomp = decomp;
	}
		
	virtual void SetId(const int id)
	{
	    m_rid = id;
	}

	/*
	virtual void SetCenter(const int id)
	{
	    m_center = id;
	    }*/
	
	virtual void SetCentroid(const double c[])
	{
	    m_centroid[0] = c[0];
	    m_centroid[1] = c[1];
	}
	
	virtual void AddNeighbor(const int id, const double cost);

	virtual int FindNeighbor(const int id)
	{
	    return FindItem<int>(&m_neighs, id);
	}
	
	virtual void SetNeighborCostByIndex(const int i, const double cost)
	{
	    m_costs[i] = cost;
	}
	
	virtual void SetNeighborCostById(const int id, const double cost)
	{
	    const int pos = FindItem<int>(&m_neighs, id);
	    if(pos >= 0)
		SetNeighborCostByIndex(pos, cost);
	}

	virtual void AddNeighborCostByIndex(const int i, const double cost)
	{
	    m_costs[i] += cost;
	}
	
	virtual void AddNeighborCostById(const int id, const double cost)
	{
	    const int pos = FindItem<int>(&m_neighs, id);
	    if(pos >= 0)
		AddNeighborCostByIndex(pos, cost);
	}
	
	
	virtual void RemoveNeighborByIndex(const int i)
	{
	    if(i < 0 && i >= m_neighs.size())
		return;

	    printf("removing neighbor %d from %d\n", m_neighs[i], GetId());
	    
	    
	    m_neighs[i] = m_neighs.back();
	    m_neighs.pop_back();
	    
	    m_costs[i] = m_costs.back();
	    m_costs.pop_back();
	}

	virtual void RemoveNeighborById(const int id)
	{
	    RemoveNeighborByIndex(FindItem<int>(&m_neighs, id));
	}
		
	virtual void AddIdTreeVertex(const int vid)
	{
	    m_vids.push_back(vid);
	}

	virtual int SelectAvailableTreeVertex(void)
	{
	    return m_vids[RandomUniformReal(0, m_vids.size() - 1)];
	}
	

	virtual void SetWeight(const double w)
	{
	    m_weight = w;

	}

	virtual void SetWeightBasedOnPathCost(void);
	
	virtual void SetNrTimesSelected(const int n)
	{
	    m_nrSels = n;
	}

	virtual void SamplePointInside(double p[])  = 0;
	
	virtual void Draw(void) = 0;
	
	virtual void DrawBoundary(Region * r) = 0;

	virtual double GetObstacleClearance(void) const
	{
	    return m_dclear;
	}

	virtual void SetObstacleClearance(const double dclear)
	{
	    m_dclear = dclear;
	}

	virtual bool IsValid(void) const
	{
	    return true;
	}

	virtual void ClearMP(void)
	{
	    m_nrSels= 0;
	    m_vids.clear();
	}
	
	
	//protected:
	Decomposition      *m_decomp;
	int                 m_rid;
	int                 m_center;
	std::vector<int>    m_neighs;
	std::vector<double> m_costs;
	double              m_centroid[2];
     	std::vector<int>    m_vids;
	double              m_weight;
	GraphPathData<int>  m_pathDataToGoal;
	int                 m_nrSels;
	double              m_area;
	double              m_dclear;

	
	
    };
    
	
}

#endif

