/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/SceneCells.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/GraphSearch.hpp"

namespace GP
{
    SceneCells::SceneCells(void)
    {
	m_grid.Setup2D(Constants::SCENE_GRID_DIMX,
		       Constants::SCENE_GRID_DIMY,
		       Constants::SCENE_GRID_MINX,
		       Constants::SCENE_GRID_MINY,
		       Constants::SCENE_GRID_MAXX,
		       Constants::SCENE_GRID_MAXY);
  }
    
    void SceneCells::SetupFromParams(Params * const p)
    {
	const int    dimsx = p->GetValueAsInt("SceneGridNrDimsX", m_grid.GetDims()[0]);
	const int    dimsy = p->GetValueAsInt("SceneGridNrDimsY", m_grid.GetDims()[1]);
	const double minx  = p->GetValueAsDouble("SceneGridMinX", m_grid.GetMin()[0]);
	const double miny  = p->GetValueAsDouble("SceneGridMinY", m_grid.GetMin()[1]);
	const double maxx  = p->GetValueAsDouble("SceneGridMaxX", m_grid.GetMax()[0]);
	const double maxy  = p->GetValueAsDouble("SceneGridMaxY", m_grid.GetMax()[1]);

	m_grid.Setup2D(dimsx, dimsy, minx, miny, maxx, maxy);
  }

    bool SceneCells::IsQuadInCollision(const double quad[]) const
    {
	//p0: 0,1
	//p1: 2, 3
	//p2: 4,5
	//p3:6,7
	const double bbox[4] =
	    {
		std::min(std::min(quad[0], quad[2]), std::min(quad[4], quad[6])),
		std::min(std::min(quad[1], quad[3]), std::min(quad[5], quad[7])),
		std::max(std::max(quad[0], quad[2]), std::max(quad[4], quad[6])),
		std::max(std::max(quad[1], quad[3]), std::max(quad[5], quad[7]))
	    };
	
	int coords_min[2];
	int coords_max[2];
	int coords[2];
	int cid;
	double min[2];
	double max[2];
	double cellQuad[8];
				
	m_grid.GetCoords(&bbox[0], coords_min);
	m_grid.GetCoords(&bbox[2], coords_max);       
	for(int x = coords_min[0]; x <= coords_max[0]; ++x)
	{
	    coords[0] = x;		    
	    for(int y = coords_min[1]; y <= coords_max[1]; ++y)
	    {
		coords[1] = y;
		cid = m_grid.GetCellIdFromCoords(coords);
		if(m_cidsObstacles.find(cid) != m_cidsObstacles.end()) //obstacle cell
		{
		    m_grid.GetCellFromCoords(coords, min, max);
		    AABoxAsPolygon2D(min, max, cellQuad);
		    if(CollisionConvexPolygons2D(4, quad, 4, cellQuad))
			return true;
		}
	    }
	}

	return false;
	
    }

	
}

