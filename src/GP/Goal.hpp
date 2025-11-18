/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP_GOAL_HPP_
#define GP_GOAL_HPP_

#include "Utils/Polygon2D.hpp"
#include "Utils/GDraw.hpp"

namespace GP
{
    class Goal
    {
    public:
	Goal(void) : m_dclear(Constants::EPSILON)
	{
	}
	
	virtual ~Goal(void)
	{
	}

	virtual bool   IsReached(const double p[])
	{
	    return m_poly.IsPointInside(p);
	}
	
	virtual void   SampleRandomPointInside(double p[])
	{
	    m_poly.SampleRandomPointInside(p);
	}
	
	virtual void   GetRepresentativePoint(double p[])
	{
	    m_poly.GetSomePointInside(p);
	}

	virtual void Read(const char fname[]);
		
	virtual void   Draw(void)
	{
	    GDrawPolygon2D(&m_poly);
	    
	}
	
	Polygon2D* GetPolygon(void)
	{
	    return &m_poly;
	}

	double GetObstacleClearance(void) const
	{
	    return m_dclear;
	}
	
	void SetObstacleClearance(const double d)
	{
	    m_dclear = d;
	}

	void SetAsCircle(const double cx, const double cy, const double r, const int nrSides)
	{
	    std::vector<double> pts(2 * nrSides);
	    
	    CircleAsPolygon2D(cx, cy, r, nrSides, &pts[0]);
	    m_poly.Clear();
	    m_poly.AddVertices(nrSides, &pts[0]);
	}
	

    protected:
	Polygon2D m_poly;
	double    m_dclear;
	
    };   

}

#endif



