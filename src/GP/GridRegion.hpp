/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__GRID_REGION_HPP_
#define GP__GRID_REGION_HPP_

#include "GP/Region.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"

namespace GP
{
    class GridRegion : public Region
    {
	friend class GridDecomposition;
	
    public:
	GridRegion(void) :
	    Region(),
	    m_parent(NULL)
	{
	}
	
	virtual ~GridRegion(void)
	{
	}
	
	virtual void SamplePointInside(double p[])
	{
	    p[0] = RandomUniformReal(m_bbox[0], m_bbox[2]);
	    p[1] = RandomUniformReal(m_bbox[1], m_bbox[3]);
	}
		
	virtual void Draw(void)
	{
	    GDrawAABox2D(m_bbox);
	}

	virtual void DrawBoundary(Region * r);

	virtual void Split(GridRegion &r1, GridRegion &r2)
	{
	    GridRegion *r;
	    
	    if(fabs(m_bbox[0] - m_bbox[2]) > fabs(m_bbox[1] - m_bbox[3]))
	    {
		r1.m_bbox[0] = m_bbox[0];
		r1.m_bbox[1] = m_bbox[1];
		r1.m_bbox[2] = 0.5 * (m_bbox[0] + m_bbox[2]);
		r1.m_bbox[3] = m_bbox[3];
		
		r2.m_bbox[0] = 0.5 * (m_bbox[0] + m_bbox[2]);
		r2.m_bbox[1] = m_bbox[1];
		r2.m_bbox[2] = m_bbox[2];		    
		r2.m_bbox[3] = m_bbox[3];
	    }
	    else
	    {
		r1.m_bbox[0] = m_bbox[0];
		r1.m_bbox[1] = m_bbox[1];
		r1.m_bbox[2] = m_bbox[2];		    
		r1.m_bbox[3] = 0.5 * (m_bbox[1] + m_bbox[3]);
		
		r2.m_bbox[0] = m_bbox[0];		    
		r2.m_bbox[1] = 0.5 * (m_bbox[1] + m_bbox[3]);
		r2.m_bbox[2] = m_bbox[2];		    
		r2.m_bbox[3] = m_bbox[3];
	    }
	    r1.m_area = r2.m_area = 0.5 * GetArea();	    
	    r1.UpdateCentroid();
	    r2.UpdateCentroid();
	    
	    for(int i = 0; i < m_cidsObstacles.size(); ++i)
	    {
		auto p = &(m_cellCentersObstacles[2 * i]);
		auto bbox = &(m_cellBoundsObstacles[4 * i]);

		if(CollisionAABoxes2D(&bbox[0], &bbox[2], &(r1.m_bbox[0]), &(r1.m_bbox[2])))
		{
		    r1.m_cidsObstacles.push_back(m_cidsObstacles[i]);
		    r1.m_cellCentersObstacles.push_back(p[0]);
		    r1.m_cellCentersObstacles.push_back(p[1]);
		    r1.m_cellBoundsObstacles.push_back(bbox[0]);
		    r1.m_cellBoundsObstacles.push_back(bbox[1]);		
		    r1.m_cellBoundsObstacles.push_back(bbox[2]);
		    r1.m_cellBoundsObstacles.push_back(bbox[3]);
		}

		if(CollisionAABoxes2D(&bbox[0], &bbox[2], &(r2.m_bbox[0]), &(r2.m_bbox[2])))
		{
		    r2.m_cidsObstacles.push_back(m_cidsObstacles[i]);
		    r2.m_cellCentersObstacles.push_back(p[0]);
		    r2.m_cellCentersObstacles.push_back(p[1]);
		    r2.m_cellBoundsObstacles.push_back(bbox[0]);
		    r2.m_cellBoundsObstacles.push_back(bbox[1]);		
		    r2.m_cellBoundsObstacles.push_back(bbox[2]);
		    r2.m_cellBoundsObstacles.push_back(bbox[3]);
		}

		/*
		if(IsPointInsideAABox2D(p, &(r1.m_bbox[0]), &(r1.m_bbox[2])))
		    r = &r1;
		else
		    r = &r2;
		r->m_cidsObstacles.push_back(m_cidsObstacles[i]);
		r->m_cellCentersObstacles.push_back(p[0]);
		r->m_cellCentersObstacles.push_back(p[1]);
		r->m_cellBoundsObstacles.push_back(bbox[0]);
		r->m_cellBoundsObstacles.push_back(bbox[1]);		
		r->m_cellBoundsObstacles.push_back(bbox[2]);
		r->m_cellBoundsObstacles.push_back(bbox[3]);*/
	    }
	}

	virtual bool IsPointInside(const double p[]) const
	{
	    return IsPointInsideAABox2D(p, &(m_bbox[0]), &(m_bbox[2]));	
	}

	virtual bool IsNeighbor(const double bbox[]) const
	{
	    auto offx = 0.001; //0.01 * (m_bbox[2] - m_bbox[0]);
	    auto offy = 0.001; //0.01 * (m_bbox[3] - m_bbox[1]);
	    const double bigger[4] = 
		{
		    m_bbox[0] - offx, m_bbox[1] - offy,
		    m_bbox[2] + offx, m_bbox[3] + offy
		};
	    
	    return CollisionAABoxes2D(&bigger[0], &bigger[2], &bbox[0], &bbox[2]);	    
	    
	}
	
	virtual bool IsBoundary(const double bmin[], const double bmax[],  double &dclear) const
	{
	    if((m_bbox[0] - Constants::EPSILON) < bmin[0])
	    {
		dclear = std::max(0.0, 0.5 * (m_bbox[0] + m_bbox[2]) - bmin[0]);
		return true;
	    }
	    if((m_bbox[2] + Constants::EPSILON) > bmax[0])
	    {
		dclear = std::max(0.0,  bmax[0] - 0.5 * (m_bbox[0] + m_bbox[2]));
		return true;
	    }

	    if((m_bbox[1] - Constants::EPSILON) < bmin[1])
	    {
		dclear = std::max(0.0, 0.5 * (m_bbox[1] + m_bbox[3]) - bmin[1]);
		return true;
	    }
	    if((m_bbox[3] + Constants::EPSILON) > bmax[1])
	    {
		dclear = std::max(0.0,  bmax[1] - 0.5 * (m_bbox[1] + m_bbox[3]));
		return true;
	    }

	    
	    return false;
	    

	    //	|| (m_bbox[2] + Constants::EPSILON) > bmax[2] ||
	    //	(m_bbox[1] - Constants::EPSILON) < bmin[1] || (m_bbox[3] + Constants::EPSILON) > bmax[3];
	}

	virtual bool IsValid(void) const
	{
	    return m_children.empty() && m_cidsObstacles.empty();
	}
	
	std::vector<GridRegion*> m_children;
	GridRegion *m_parent;
	std::vector<int> m_cidsObstacles;
	std::vector<double> m_cellCentersObstacles;
	std::vector<double> m_cellBoundsObstacles;
	
	//protected:
	virtual void UpdateCentroid(void)
	{
	    m_centroid[0] = 0.5 * (m_bbox[0] + m_bbox[2]);
	    m_centroid[1] = 0.5 * (m_bbox[1] + m_bbox[3]); 
	}
	
	double m_bbox[4];
	
    };
    
	
}

#endif

