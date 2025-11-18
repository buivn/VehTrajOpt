/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "Utils/Polygon2D.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/PrintMsg.hpp"
#include "External/ShewchukTriangle.hpp"

namespace GP
{
    void Polygon2D::Clear(void)
    {
	m_flags = RECOMPUTE_TRIANGULATION | RECOMPUTE_CENTROID | RECOMPUTE_CONVEXITY | RECOMPUTE_BOUNDING_BOX | RECOMPUTE_TRI_AREAS;
	m_bbox[0] = m_bbox[1] = m_bbox[2] = m_bbox[3] = 0;
	m_vertices.clear();
	m_triIndices.clear();
	m_triAreas.clear();
	m_area = 0.0;
	m_centroid[0] = m_centroid[1] = 0.0;
	m_isConvex = true;
    }
    
	    
    int Polygon2D::GetNrVertices(void) const
    {
	return m_vertices.size() / 2;
    }

    double Polygon2D::GetVertexX(const int i) const
    {
	return m_vertices[2 * i];
    }
    
    double Polygon2D::GetVertexY(const int i) const
    {
	return m_vertices[2 * i + 1];
    }
    
    void Polygon2D::GetVertex(const int i, double v[2]) const
    {
	v[0] = GetVertexX(i);
	v[1] = GetVertexY(i);
    }

    void Polygon2D::AddVertex(const double v[2])
    {
	AddVertex(v[0], v[1]);
    }
    
    void Polygon2D::AddVertex(const double vx, const double vy)
    {
	m_vertices.push_back(vx);
	m_vertices.push_back(vy);
	OnShapeChange();
    }

    
    void Polygon2D::AddVertices(const int n, const double vertices[])
    {
	for(int i = 0; i < n; ++i)
	    AddVertex(vertices[2 * i], vertices[2 * i + 1]);
    }
        
    void Polygon2D::SetVertex(const int i, const double v[2])
    {
	SetVertex(i, v[0], v[1]);
    }

    void Polygon2D::SetVertex(const int i, const double vx, const double vy)
    {
	m_vertices[2 * i] = vx;
	m_vertices[2 * i + 1] = vy;
	OnShapeChange();
    }
    
    void Polygon2D::MakeCCW(void)
    {
//polygon is ccw if signed area is positive
	const int n = GetNrVertices();
	double    a = 0;
	int       inext;
	for(int i = 0; i < n; ++i)
	{
	    inext = (i + 1) % n;
	    a    += m_vertices[(i << 1)] * m_vertices[(inext << 1) + 1] - m_vertices[(inext << 1)] * m_vertices[(i << 1) + 1];
	}

	if(a < 0)
	{
//polygon is not ccw, so change its orientation
	    double t;	    
	    for(int i = 0, j = 2 * n - 2; i < j; i += 2, j -= 2)
	    {
		t = m_vertices[i];     m_vertices[i]     = m_vertices[j];      m_vertices[j]     = t;
		t = m_vertices[i + 1]; m_vertices[i + 1] = m_vertices[j + 1];  m_vertices[j + 1] = t;
	    }   
	}
	m_flags = AddFlag(m_flags, RECOMPUTE_TRIANGULATION | RECOMPUTE_TRI_AREAS);
    }

    
    const double* Polygon2D::GetBoundingBox(void)
    {
	if(HasFlag(m_flags, RECOMPUTE_BOUNDING_BOX))
	{
	    m_flags = RemoveFlag(m_flags, RECOMPUTE_BOUNDING_BOX);

	    const int n = GetNrVertices();
	    if(n == 0)
		return m_bbox;
	    	    
	    m_bbox[0] = m_bbox[2] = GetVertexX(0);
	    m_bbox[1] = m_bbox[3] = GetVertexY(0);
	    
	    for(int i = 1; i < n; ++i)
	    {
		const double x = GetVertexX(i);
		const double y = GetVertexY(i);
		
		if(x < m_bbox[0])       m_bbox[0] = x;
		else if(x > m_bbox[2])  m_bbox[2] = x;
		
		if(y < m_bbox[1])       m_bbox[1] = y;
		else if(y > m_bbox[3])  m_bbox[3] = y;
	    }
	}
	
	return m_bbox;
    }
    
    bool Polygon2D::IsConvex(void)
    {
	if(HasFlag(m_flags, RECOMPUTE_CONVEXITY))
	{
	    m_flags = RemoveFlag(m_flags, RECOMPUTE_CONVEXITY);
	    
	    const int n = GetNrVertices();
	    
	    if(n <= 3) 
		m_isConvex = true;
	    else
	    {
		const bool turn = Turn2D(&m_vertices[0], &m_vertices[2], &m_vertices[4]) <= Constants::EPSILON;
		
		for(int i = 1; i < n; i++)
		    if(turn != (Turn2D(&m_vertices[2 * i], 
				       &m_vertices[2 * ((i + 1) % n)],
				       &m_vertices[2 * ((i + 2) % n)]) <= Constants::EPSILON))
		    {
			m_isConvex = false;
			return m_isConvex;
		    }
		m_isConvex = true;
	    }
	}

	return m_isConvex;
    }
    
    void Polygon2D::Triangulate(void)
    {
	if(HasFlag(m_flags, RECOMPUTE_TRIANGULATION))
	{
	
	    m_flags = RemoveFlag(m_flags, RECOMPUTE_TRIANGULATION);
	    m_triIndices.clear();		
	    TriangulatePolygonWithNoHoles2D(false, -1, -1,
					    m_vertices.size() / 2, &m_vertices[0],
					    NULL, &m_triIndices, NULL);

	    for(auto & val : m_triIndices)
		if(val >= (m_vertices.size() / 2))
		    printf("..messed up tri = %d vertices = %d\n", val, m_vertices.size() / 2);
	    
	}
    }
        
    int Polygon2D::GetNrTriangles(void)
    {
	Triangulate();
	return m_triIndices.size() / 3;
    }

    void Polygon2D::GetTriangleIndices(const int i, int tri[3])
    {
	Triangulate();
	tri[0] = m_triIndices[3 * i];
	tri[1] = m_triIndices[3 * i + 1];
	tri[2] = m_triIndices[3 * i + 2];
    }

    
    void Polygon2D::GetTriangleVertices(const int i, double tri[6])
    {
	int tind[3];
	GetTriangleIndices(i, tind);
	GetVertex(tind[0], &tri[0]);
	GetVertex(tind[1], &tri[2]);
	GetVertex(tind[2], &tri[4]);
    }
    
    const double* Polygon2D::GetCentroid(void)
    {
	if(HasFlag(m_flags, RECOMPUTE_CENTROID))
	{
	    m_flags = RemoveFlag(m_flags, RECOMPUTE_CENTROID);
	    
	    const int n  = GetNrVertices();
	    double    sx = 0;
	    double    sy = 0;
	    double    v[2];
	    
	    for(int i = 0; i < n; ++i)
	    {
		GetVertex(i, v);
		sx += v[0];
		sy += v[1];
	    }
	    m_centroid[0] = sx / n;
	    m_centroid[1] = sy / n;
	}
	return m_centroid;
    }
    
    

    void Polygon2D::GetSomePointInside(double p[2])
    {
	if(IsConvex())
	{
	    const double *c = GetCentroid();
	    p[0] = c[0];
	    p[1] = c[1];
	}
	else
	{
	    double tri[6];	
	    GetTriangleVertices(0, tri);
	    const double a1 = 0.35;
	    const double a2 = 0.45;
	    
	    p[0] = (tri[0] + tri[2] + tri[4]) / 3.0;
	    p[1] = (tri[1] + tri[3] + tri[5]) / 3.0;
	}
    }

    void Polygon2D::Read(FILE *in)
    {
	int n;
	if(fscanf(in, "%d", &n) != 1)
	    OnInputError(printf("expecting number of vertices\n"));
	
	m_vertices.resize(2 * n);
	for(int i = 0; i < 2 * n; ++i)
	{
	    if(fscanf(in, "%lf", &m_vertices[i]) != 1)
		OnInputError(printf("expecting %d-th value for polygon with %d vertices\n", i, n));
	}
	
	OnShapeChange();
	MakeCCW();
    }

    bool Polygon2D::IsPointInside(const double p[])
    {
	const int n = GetNrVertices();
	
	if(n <= 2)
	    return false;

	const double *bbox = GetBoundingBox();
	
	if(IsPointInsideAABox2D(p, &bbox[0], &bbox[2]) == false)
	    return false;
		
	if(IsConvex())
	    return IsPointInsideConvexPolygon2D(p, n, &m_vertices[0]);
	else
	    return IsPointInsidePolygon2D(p, n, &m_vertices[0]);
	
    }
    
    bool Polygon2D::SegmentCollision(const double p1[], const double p2[])
    {
	double pinter[2];
	
	const int n = GetNrVertices();
	
	if(n <= 1)
	    return false;
	if(n == 2)
	    return IntersectSegments2D(p1[0], p1[1], p2[0], p2[1],
				       GetVertexX(0), GetVertexY(0),
				       GetVertexX(1), GetVertexY(1),
				       &pinter[0], &pinter[1]);


	if(IsConvex())
	    return CollisionSegmentConvexPolygon2D(p1, p2, n, &m_vertices[0]);
	else
	    return CollisionSegmentPolygon2D(p1, p2, n, &m_vertices[0]);
    }

    double Polygon2D::PointDistance(const double p[], double pmin[])
    {
	return sqrt(DistSquaredPointPolygon2D(p,
					      GetNrVertices(),
					      &m_vertices[0],
					      pmin));
    }
    
    
    double Polygon2D::PolygonDistance(Polygon2D *poly)
    {
	double pmin1[2];
	double pmin2[2];
	
	return sqrt(DistSquaredPolygons2D(GetNrVertices(), 
					  &m_vertices[0],
					  poly->GetNrVertices(),
					  &(*(poly->GetVertices()))[0],
					  pmin1,
					  pmin2));
    }
    
    bool Polygon2D::PolygonCollision(Polygon2D *poly)
    {
	const bool c1 = IsConvex();
	const bool c2 = poly->IsConvex();

	const double *v2 = &(*(poly->GetVertices()))[0];
	
	
	
	if(c1 && c2)
	    return CollisionConvexPolygons2D(GetNrVertices(), &m_vertices[0],
					     poly->GetNrVertices(), v2);
	else if(c1)
	    return CollisionPolygonConvexPolygon2D(poly->GetNrVertices(), v2,
						   GetNrVertices(), &m_vertices[0]);
	else if(c2)
	    return CollisionPolygonConvexPolygon2D(GetNrVertices(), &m_vertices[0],
						   poly->GetNrVertices(), v2);
	else
	    return CollisionPolygons2D(GetNrVertices(), &m_vertices[0],
				       poly->GetNrVertices(), v2);
    }
    

    
    void Polygon2D::Print(FILE * out) const
    {
	const int n = GetNrVertices();
	
	fprintf(out, "%d ", n);
	for(int i = 0; i < n; ++i)
	    fprintf(out, "%f %f  ", GetVertexX(i), GetVertexY(i));
//	fprintf(out, "\n");
    }

    void ReadPolygons2D(FILE * in, std::vector<Polygon2D*> * const polys)
    {
	Polygon2D *poly;
	int        n;
	
	if(fscanf(in, "%d", &n) != 1)
	    OnInputError(printf("expecting number of polygons\n"));
	
	for(int i = 0; i < n; ++i)
	{
	    poly = new Polygon2D();
	    poly->Read(in);
	    polys->push_back(poly);
	}
	printf("...........read %d polygons\n", n);
	
    }

    
    void Polygon2D::OccupiedGridCells(const Grid * const       grid, 
				      std::vector<int> * const cellsInside,
				      std::vector<int> * const cellsIntersect)
    {		
	const double *bbox = GetBoundingBox();
	const int     n    = m_vertices.size() / 2;
	const double *poly = &m_vertices[0];
	const bool    checkIntersection = false;
	
	int coord_min[3]={0,0,0};
	int coord_max[3] ={0,0,0};
	int coords[3] = {0,0,0};
	double min[3], max[3];
	
	cellsInside->clear();
	cellsIntersect->clear();		
	grid->GetCoords(&bbox[0], coord_min);
	grid->GetCoords(&bbox[2], coord_max);       
	for(int x = coord_min[0]; x <= coord_max[0]; ++x)
	{
	    coords[0] = x;		    
	    for(int y = coord_min[1]; y <= coord_max[1]; ++y)
	    {
		coords[1] = y;
		
		grid->GetCellFromCoords(coords, min, max);
		double box[8];
		AABoxAsPolygon2D(min, max, box);
		
		if(IntersectPolygons2D(4, box, n, poly))
		    cellsIntersect->push_back(grid->GetCellIdFromCoords(coords));
		else if(IsPolygonInsidePolygon2D(4, box, n, poly, checkIntersection))
		    cellsInside->push_back(grid->GetCellIdFromCoords(coords));
		else if(IsPolygonInsideAABox2D(n, poly, min, max))
		    cellsIntersect->push_back(grid->GetCellIdFromCoords(coords));
			
	    }
	    
	}
    }   

    double Polygon2D::GetArea(void)
    {
	if(HasFlag(m_flags, RECOMPUTE_TRI_AREAS))
	{
	    m_flags = RemoveFlag(m_flags, RECOMPUTE_TRI_AREAS);
	    
	    m_triAreas.clear();
	    const int n = GetNrTriangles();
	    double tri[6];
	    double area = 0;
	    
	    m_triAreas.resize(n);
	    m_area = 0;
	    for(int i = 0; i < n; ++i)
	    {
		GetTriangleVertices(i, tri);
		m_triAreas[i] = fabs(SignedAreaPolygon2D(3, tri));
		m_area += m_triAreas[i];
	    }
	}
	
	return m_area;
    }

    
   void Polygon2D::SampleRandomPointInside(double p[])
    {
	const int t = SelectTriangleBasedOnArea();
	SampleRandomPointInsideTriangle2D(&m_vertices[2 * m_triIndices[3 * t]],
					  &m_vertices[2 * m_triIndices[3 * t + 1]],
					  &m_vertices[2 * m_triIndices[3 * t + 2]], p);
  }

    int Polygon2D::SelectTriangleBasedOnArea(void)
    {
	const int    n = GetNrTriangles();
	const double r = RandomUniformReal(0, GetArea());
	double       w = 0;
	
	for(int i = 0; i < n; ++i)
	{
	    w += m_triAreas[i];
	    if(w >= r)
		return i;
	}
	return n - 1;
    }

    int Polygon2D::LocateTriangle(const double p[])
    {
	double tri[6];
	
	for(int i = GetNrTriangles() - 1; i >= 0; --i)
	{
	    GetTriangleVertices(i, tri);
	    if(IsPointInsideTriangle2D(p, &tri[0], &tri[2], &tri[4]))
		return i;
	}
	return Constants::ID_UNDEFINED;
    }
    
}	



