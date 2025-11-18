/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/ScenePolys.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/TriMeshReader.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/PrintMsg.hpp"
#include "Utils/GDraw.hpp"
#include "TrajOpt/DevPlanner.hpp"

namespace GP
{
    ScenePolys::ScenePolys(void)
    {
	m_grid.Setup2D(Constants::SCENE_GRID_DIMX,
		       Constants::SCENE_GRID_DIMY,
		       Constants::SCENE_GRID_MINX,
		       Constants::SCENE_GRID_MINY,
		       Constants::SCENE_GRID_MAXX,
		       Constants::SCENE_GRID_MAXY);

	m_hObstacles = 1.0;//Constants::SCENE_OBSTACLES_HEIGHT;
	m_hBoundary  = 1.0;//Constants::SCENE_BOUNDARY_HEIGHT;	
    }
    
    void ScenePolys::SetupFromParams(Params * const p)
    {
	const int    dimsx = p->GetValueAsInt("SceneGridNrDimsX", m_grid.GetDims()[0]);
	const int    dimsy = p->GetValueAsInt("SceneGridNrDimsY", m_grid.GetDims()[1]);
	const double minx  = p->GetValueAsDouble("SceneGridMinX", m_grid.GetMin()[0]);
	const double miny  = p->GetValueAsDouble("SceneGridMinY", m_grid.GetMin()[1]);
	const double maxx  = p->GetValueAsDouble("SceneGridMaxX", m_grid.GetMax()[0]);
	const double maxy  = p->GetValueAsDouble("SceneGridMaxY", m_grid.GetMax()[1]);

	m_grid.Setup2D(dimsx, dimsy, minx, miny, maxx, maxy);

	//m_hObstacles = p->GetValueAsDouble("SceneObstaclesHeight", m_hObstacles);
	//m_hBoundary  = p->GetValueAsDouble("SceneBoundaryHeight", m_hBoundary);
    }

    void ScenePolys::AddObstacle(Polygon2D * const poly)
    {
	m_obstacles.push_back(poly);
	m_tmeshObstacles.AddPolygon(poly);
	m_tmeshObstaclesForDrawing.AddExtrudedPolygon(poly, 0, m_hObstacles);
    }

    void ScenePolys::AddCellObstacle(const int cid)
    {
	if(m_cidsObstacles.find(cid) != m_cidsObstacles.end())
	    return;
	
	m_cidsObstacles.insert(cid);
	
	
	double pmin[2];
	double pmax[2];
	auto poly = new Polygon2D();
	
	m_grid.GetCellFromId(cid, pmin, pmax);
	poly->AddVertex(pmin[0], pmin[1]);
	poly->AddVertex(pmax[0], pmin[1]);
	poly->AddVertex(pmax[0], pmax[1]);
	poly->AddVertex(pmin[0], pmax[1]);
	AddObstacle(poly);
    }
    
    /*
    void ScenePolys::ReadObstacles(const char fname[])
    {	
	FILE *in = fopen(fname, "r");
	if(!in)
	{
	    // OnInputError(printf("could not open <%s> for reading\n", fname));
	    return;
	    
	}

	std::vector<Polygon2D*> obstacles;
	std::vector<int> cellsInside;
	std::vector<int> cellsIntersect;
	
	ReadPolygons2D(in, &obstacles);
	fclose(in);

	m_tmeshObstacles.Clear();
	m_tmeshObstaclesForDrawing.Clear();	
	m_obstacles.clear();

	for(int i = obstacles.size() - 1; i >= 0; --i)
	{
	    obstacles[i]->OccupiedGridCells(&m_grid, &cellsInside, &cellsIntersect);
	    for(auto &cid : cellsInside)
		AddCellObstacle(cid);
	    for(auto &cid : cellsIntersect)
	    	AddCellObstacle(cid);
	    delete obstacles[i];	    
	}
	

	printf("added obstacles: %d\n", m_obstacles.size());	
    }
    */
    

   void ScenePolys::ReadObstacles(const char fname[])
    {
	std::vector<int> cellsInside;
	std::vector<int> cellsIntersect;
	
	FILE *in = fopen(fname, "r");
	if(!in)
	{
	    //OnInputError(printf("could not open <%s> for reading\n", fname));
	    return;
	    
	}

	ReadPolygons2D(in, &m_obstacles);
	fclose(in);

	m_tmeshObstacles.Clear();
	for(int i = m_obstacles.size() - 1; i >= 0; --i)
	{
	    m_tmeshObstacles.AddPolygon(m_obstacles[i]);
	    m_tmeshObstaclesForDrawing.AddExtrudedPolygon(m_obstacles[i], 0, m_hObstacles);
	}
	
	printf("added obstacles: %d\n", m_obstacles.size());

	for(int i = m_obstacles.size() - 1; i >= 0; --i)
	{
	    m_obstacles[i]->OccupiedGridCells(&m_grid, &cellsInside, &cellsIntersect);
	    for(auto &cid : cellsInside)
		m_cidsObstacles.insert(cid);
	    for(auto &cid : cellsIntersect)
	    	m_cidsObstacles.insert(cid);
	    
	}

    }

    
   void ScenePolys::ReadOccupancyGrid(const char fname[])
    {
	FILE *in = fopen(fname, "r");
	if(!in)
	{
	    return;
	}
	int val;
	int cid = 0;
	//int coords[2];
	//int nrDims = m_grid.GetDims()[1];

	//printf("nrDims = %d\n", nrDims);
	
	while(fscanf(in, "%d", &val) == 1)
	{
	    //coords[0] = cid/ nrDims;
	    //coords[1] = cid% nrDims;
	    auto use_cid = cid;//m_grid.GetCellIdFromCoords(coords);
	    
	    if(val == GRID_CELL_OBSTACLE)
		AddCellObstacle(use_cid);
	    else if(val == GRID_CELL_UNKNOWN)
		m_cidsUnknown.insert(use_cid);
	    else if(val == GRID_CELL_FREE)
		m_cidsFree.insert(use_cid);	    
	    ++cid;
	}
	fclose(in);

	printf("read %d cells -- %d obstacles, %d unknown, %d free : total = %d %d\n", cid, m_cidsObstacles.size(), m_cidsUnknown.size(), m_cidsFree.size(), m_grid.GetNrCells(), m_cidsObstacles.size() + m_cidsUnknown.size() + m_cidsFree.size());

    }

    void ScenePolys::ReadTerrain(const char fname[])
    {	
	TriMeshReader(fname, &m_tmeshTerrain);
    }

    void ScenePolys::PrintObstacles(const char fname[])
    {
	FILE *out = fopen(fname, "w");
	fprintf(out, "%d\n", m_obstacles.size());
	for(int i = 0; i < (int) m_obstacles.size(); ++i)
	    m_obstacles[i]->Print(out);
	fclose(out);
    }
    

    bool ScenePolys::SegmentObstaclesCollision(const double p1[2], const double p2[2])
    {	
	const int n = m_obstacles.size();
	for(int i = 0; i < n; ++i)
	    if(m_obstacles[i]->SegmentCollision(p1, p2))
		return true;
	return false;
    }

    void ScenePolys::DrawObstacles(void)
    {
	m_tmeshObstaclesForDrawing.Draw();
	//m_tmeshObstacles.Draw();
	
	//for(int i = m_obstacles.size() - 1; i >= 0; --i)
	//  GDrawPolygon2D(m_obstacles[i]);


	
    }

    void ScenePolys::DrawTerrain(void)
    {
	m_tmeshTerrain.Draw();
    }
    

    void ScenePolys::AdjustGrid(const double eps)
    {
	double pmin[2];
	double pmax[2];
	
	pmin[0] = pmin[1] = HUGE_VAL;
	pmax[0] = pmax[1] = -HUGE_VAL;
	
	for(int i = m_obstacles.size() - 1; i >= 0; --i)
	{
	    const double *bbox = m_obstacles[i]->GetBoundingBox();
	    if(bbox[0] < pmin[0])
		pmin[0] = bbox[0];
	    if(bbox[1] < pmin[1])
		pmin[1] = bbox[1];
	    if(bbox[2] > pmax[0])
		pmax[0] = bbox[2];
	    if(bbox[3] > pmax[1])
		pmax[1] = bbox[3];
	    
	}
	pmin[0] -= eps;
	pmin[1] -= eps;
	pmax[0] += eps;
	pmax[1] += eps;

	const double *gmin = m_grid.GetMin();
	const double *gmax = m_grid.GetMax();
	
	if(gmin[0] < pmin[0])
	    pmin[0] = gmin[0];
	if(gmin[1] < pmin[1])
	    pmin[1] = gmin[1];
	
	if(gmax[0] > pmax[0])
	    pmax[0] = gmax[0];
	if(gmax[1] > pmax[1])
	    pmax[1] = gmax[1];
	
	m_grid.Setup(2, m_grid.GetDims(), pmin, pmax);
       
    }

    void ScenePolys::AddBoundary(const double thick)
    {
	const double *pmin = m_grid.GetMin();
	const double *pmax = m_grid.GetMax();
	
	double     poly[8];	
	Polygon2D *bound;

	AABoxAsPolygon2D(pmin[0], pmin[1] - 0.5*thick,  pmax[0], pmin[1] + 0.5*thick, poly);
	bound = new Polygon2D();
	bound->AddVertex(poly[0], poly[1]);
	bound->AddVertex(poly[2], poly[3]);
	bound->AddVertex(poly[4], poly[5]);
	bound->AddVertex(poly[6], poly[7]);
	m_bounds.push_back(bound);

	AABoxAsPolygon2D(pmin[0], pmax[1] - 0.5*thick,  pmax[0], pmax[1] + 0.5*thick, poly);
	bound = new Polygon2D();
	bound->AddVertex(poly[0], poly[1]);
	bound->AddVertex(poly[2], poly[3]);
	bound->AddVertex(poly[4], poly[5]);
	bound->AddVertex(poly[6], poly[7]);
	m_bounds.push_back(bound);

	AABoxAsPolygon2D(pmin[0] - 0.5 * thick, pmin[1] + 0.5 * thick,  pmin[0] + 0.5*thick, pmax[1] - 0.5 * thick, poly);
	bound = new Polygon2D();
	bound->AddVertex(poly[0], poly[1]);
	bound->AddVertex(poly[2], poly[3]);
	bound->AddVertex(poly[4], poly[5]);
	bound->AddVertex(poly[6], poly[7]);
	m_bounds.push_back(bound);


	AABoxAsPolygon2D(pmax[0] - 0.5 * thick, pmin[1] + 0.5 * thick,  pmax[0] + 0.5*thick, pmax[1] - 0.5 * thick, poly);
	bound = new Polygon2D();
	bound->AddVertex(poly[0], poly[1]);
	bound->AddVertex(poly[2], poly[3]);
	bound->AddVertex(poly[4], poly[5]);
	bound->AddVertex(poly[6], poly[7]);
	m_bounds.push_back(bound);

	for(int i = m_bounds.size() - 1; i >= 0; --i)
	{
	    m_tmeshObstacles.AddPolygon(m_bounds[i]);
	    m_tmeshObstaclesForDrawing.AddExtrudedPolygon(m_bounds[i], 0, m_hBoundary);
	}

	std::vector<int> cellsInside;
	std::vector<int> cellsIntersect;	
	for(int i = m_bounds.size() - 1; i >= 0; --i)
	{
	    m_bounds[i]->OccupiedGridCells(&m_grid, &cellsInside, &cellsIntersect);
	    for(auto &cid : cellsInside)
		m_cidsObstacles.insert(cid);
	    for(auto &cid : cellsIntersect)
	    	m_cidsObstacles.insert(cid);
	    
	}

    }

    //////////////////
    bool ScenePolys::IsInCollision(Polygon2D &poly) const
    {
	PQPTriMesh tmesh;	
	tmesh.AddPolygon(&poly);
	
	return GetTriMeshObstacles()->Collision(NULL, NULL, &tmesh, NULL, NULL);
	    
	std::vector<int> cidsInside;
	std::vector<int> cidsIntersect;
	
	poly.OccupiedGridCells(&m_grid, &cidsInside, &cidsIntersect);

	for(auto & cid : cidsInside)
	    if(m_cidsObstacles.find(cid) != m_cidsObstacles.end())
		return true;
	for(auto & cid : cidsIntersect)
	    if(m_cidsObstacles.find(cid) != m_cidsObstacles.end())
		return true;
	return false;
    }

	
}

