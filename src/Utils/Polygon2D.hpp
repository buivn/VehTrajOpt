/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__POLYGON2D_HPP_
#define GP__POLYGON2D_HPP_

#include "Utils/Flags.hpp"
#include "Utils/Grid.hpp"
#include "Utils/Geometry.hpp"
#include <vector>
#include <cstdio>


namespace GP
{
    class Polygon2D
    {
    public:
	Polygon2D(void)
	{
	    Clear();
	}
	
		
	virtual ~Polygon2D(void)
	{
	}

	virtual int GetNrVertices(void) const;
	
	virtual void GetVertex(const int i, double v[2]) const;

	virtual double GetVertexX(const int i) const;
	
	virtual double GetVertexY(const int i) const;

	virtual const std::vector<double>* GetVertices(void) const
	{
	    return &m_vertices;
	}
		
	virtual void AddVertex(const double v[2]);
	
	virtual void AddVertex(const double vx, const double vy);
	
	virtual void AddVertices(const int n, const double vertices[]);
	
	virtual void SetVertex(const int i, const double v[2]);
	
	virtual void SetVertex(const int i, const double vx, const double vy);
		
	virtual void MakeCCW(void);
		
	virtual bool IsConvex(void);
		
	virtual int GetNrTriangles(void);
	
	virtual void GetTriangleIndices(const int i, int tri[3]);

	virtual void GetTriangleVertices(const int i, double tri[6]);
		
	virtual const double* GetCentroid(void);
		
	virtual void GetSomePointInside(double p[2]);

	virtual void SampleRandomPointInside(double p[2]);
	
	virtual const double* GetBoundingBox(void);
	
	virtual bool IsPointInside(const double p[]);

	virtual bool SegmentCollision(const double p1[], const double p2[]);

	virtual double PolygonDistance(Polygon2D *poly);
	
	virtual bool PolygonCollision(Polygon2D *poly);

	virtual double PointDistance(const double p[], double pmin[]);
					     
	
	virtual void  OccupiedGridCells(const Grid * const       grid, 
					std::vector<int> * const cellsInside,
					std::vector<int> * const cellsIntersect);
	

	virtual int LocateTriangle(const double p[]);
	

	virtual double GetArea(void);

	int SelectTriangleBasedOnArea(void);

	
	virtual void Read(FILE * in);
	
	virtual void Print(FILE * out) const;

	virtual void Clear(void);

	
	virtual void ApplyTransRot(const double TR[])
	{
	    ApplyTransRotToPolygon2D(TR, m_vertices.size() / 2,
				     &m_vertices[0],
				     &m_vertices[0]);
	    OnShapeChange();
	    
	}
    	
	
    protected:
	virtual void Triangulate(void);
	
	virtual void OnShapeChange(void)
	{
	    m_flags = AddFlag(m_flags, 
			      RECOMPUTE_TRIANGULATION | 
			      RECOMPUTE_CENTROID | 
			      RECOMPUTE_CONVEXITY |
			      RECOMPUTE_BOUNDING_BOX |
			      RECOMPUTE_TRI_AREAS);
	}

	std::vector<double> m_vertices;
	std::vector<int>    m_triIndices;
	std::vector<double> m_triAreas;
	double              m_area;
	double              m_centroid[2];
	bool                m_isConvex;	
	double              m_bbox[4];
	Flags               m_flags;

	enum
	    {
		RECOMPUTE_TRIANGULATION  = 1,
		RECOMPUTE_CENTROID       = 2,
		RECOMPUTE_CONVEXITY      = 4,
		RECOMPUTE_BOUNDING_BOX   = 8,
		RECOMPUTE_TRI_AREAS      = 16
	    };
    };

    void ReadPolygons2D(FILE * in, std::vector<Polygon2D*> * const polys);
    
  
}

#endif
    
    
    
