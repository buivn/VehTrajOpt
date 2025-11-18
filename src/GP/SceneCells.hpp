/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__SCENE_CELLS_HPP_
#define GP__SCENE_CELLS_HPP_

#include "Utils/Grid.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include <unordered_set>

namespace GP
{
    class SceneCells
    {
    public:
	SceneCells(void);
	
	virtual ~SceneCells(void)
	{
	}

	virtual void SetupFromParams(Params * const p);
	
	virtual Grid* GetGrid(void)
	{
	    return &m_grid;
	}
	
	virtual const Grid* GetGrid(void) const
	{
	    return &m_grid;
	}
		
	virtual bool IsQuadInCollision(const double quad[]) const;
	
	virtual void AddCellFree(const int cid)
	{
	    m_cidsFree.insert(cid);
	}
	

	virtual void AddCellUnknown(const int cid)
	{
	    m_cidsUnknown.insert(cid);
	}
	

	virtual void AddCellObstacle(const int cid)
	{
	    m_cidsObstacles.insert(cid);
	}
	

	virtual void ResetForNewMP(void)
	{
	    m_cidsObstacles.clear();
	    m_cidsUnknown.clear();
	    m_cidsFree.clear();	    	    
	}
	
	Grid  m_grid;
	std::unordered_set<int> m_cidsObstacles;
	std::unordered_set<int> m_cidsUnknown;
	std::unordered_set<int> m_cidsFree;
    };
}

#endif



