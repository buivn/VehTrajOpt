/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__SCENE_POLYS_HPP_
#define GP__SCENE_POLYS_HPP_

#include "Utils/Grid.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include <vector>
#include <unordered_set>

namespace GP
{
    class ScenePolys
    {
    public:
	ScenePolys(void);
	
	virtual ~ScenePolys(void)
	{
	    DeleteItems<Polygon2D*>(&m_obstacles);
	    DeleteItems<Polygon2D*>(&m_bounds);
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

	virtual void ReadOccupancyGrid(const char fname[]);
	
		
	virtual void ReadObstacles(const char fname[]);

	virtual void ReadTerrain(const char fname[]);
	
	virtual void PrintObstacles(const char fname[]);
	
	virtual int GetNrObstacles(void) const
	{
	    return m_obstacles.size();
	}
	
	virtual const Polygon2D* GetObstacle(const int i) const
	{
	    return m_obstacles[i];
	}

	virtual Polygon2D* GetObstacle(const int i)
	{
	    return m_obstacles[i];
	}

	virtual bool SegmentObstaclesCollision(const double p1[2], const double p2[2]);
	
	virtual void DrawObstacles(void);

	virtual void DrawTerrain(void);
	
	virtual TriMesh* GetTriMeshObstacles(void)
	{
	    return &m_tmeshObstacles;
	}

	virtual void AdjustGrid(const double eps);

	virtual void AddBoundary(const double thick);

	virtual void AddObstacle(Polygon2D * const poly);

	virtual void AddCellObstacle(const int cid);
	
	
	virtual TriMesh* GetTerrain(void) 
	{
	    return &m_tmeshTerrain;
	}

	virtual void ResetForMP(void)
	{
	    RemoveObstacles();
	    m_cidsObstacles.clear();
	}
	

	virtual void RemoveObstacles(void)
	{
	    DeleteItems<Polygon2D*>(&m_obstacles);
	    m_obstacles.clear();
	    m_tmeshObstacles.Clear();
	    m_tmeshObstaclesForDrawing.Clear();
	}

	virtual bool IsInCollision(Polygon2D &poly) const;

	virtual bool IsCellInCollision(const int cid) const
	{
	    /*PQPTriMesh tmesh;
	    double bbox[4];
	    double quad[12];
	    
	    m_grid.GetCellFromId(cid, bbox);
	    quad[0] = bbox[0]; quad[1] = bbox[1]; quad[2] = 0;
	    quad[3] = bbox[2]; quad[4] = bbox[1]; quad[5] = 0;
	    quad[6] = bbox[2]; quad[7] = bbox[3]; quad[8] = 0;
	    quad[9] = bbox[0]; quad[10] = bbox[3]; quad[11] = 0;
	    tmesh.AddQuad(quad);
	    return GetTriMeshObstacles()->Collision(NULL, NULL, &tmesh, NULL, NULL);
	    */
	    

	    return m_cidsObstacles.find(cid) != m_cidsObstacles.end();
	}

	Grid   m_grid;
	std::unordered_set<int> m_cidsObstacles;
	std::unordered_set<int> m_cidsFree;
	std::unordered_set<int> m_cidsUnknown;
	
	
	std::vector<Polygon2D*> m_obstacles;
	std::vector<Polygon2D*> m_bounds;
	PQPTriMesh              m_tmeshTerrain;
	PQPTriMesh              m_tmeshObstacles;
	TriMesh                 m_tmeshObstaclesForDrawing;
	double                  m_hObstacles;
	double                  m_hBoundary;
    };
}

#endif



