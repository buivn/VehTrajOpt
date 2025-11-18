/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/Follow.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"

namespace GP
{
    Follow::Follow(void)
    {
	m_wbase    = Constants::FOLLOW_WEIGHT_BASE;
	m_tolReach = Constants::FOLLOW_REACH_TOLERANCE;
	
	Clear();
    }
		
    void Follow::SetupFromParams(Params * const p)
    {
	m_wbase    = p->GetValueAsDouble("FollowWeightBase", m_wbase);
	m_tolReach = p->GetValueAsDouble("FollowReachTolerance", m_tolReach);
    }
    
    void Follow::Clear(void)
    {
	m_waypts.clear();
    }
    
    void Follow::AddWaypt(const double px, const double py, const double r)
    {
	m_waypts.push_back(px);
	m_waypts.push_back(py);
	m_waypts.push_back(r);
    }
    

    bool Follow::IsInside(const int i, const double p[]) const
    {
	return IsPointInsideConvexPolygon2D(p, 4, &m_quads[8*i]);
    
	
	double        pmin[2];
	const double *wpt = GetWaypt(i);

	if(i == 0)
	    return Algebra2D::PointDistSquared(p, wpt) <= wpt[2] * wpt[2];
	else
	    return DistSquaredPointSegment2D(p, GetWaypt(i-1), wpt, pmin) <= wpt[2] * wpt[2];

    }
    

    bool Follow::Reached(const int i, const double p[]) const
    {
	//return IsPointInsideConvexPolygon2D(p, 4, &m_goals[8 * i]);
	
	return Algebra2D::PointDistSquared(GetWaypt(i), p) <= m_tolReach * m_tolReach;
    }


    int Follow::Nearest(const int k, const double p[]) const
    {	
	const int n    = GetNrWaypts();
	int       imin = -1;
	double    dmin = HUGE_VAL;
	double    d;
	
	for(int i = std::max(0, k - 1); i <= k; ++i)
	    if((d = Algebra2D::PointDistSquared(p, GetWaypt(i))) < dmin)
	    {	
		dmin = d;
		imin = i;
	    }
	return imin;
    }

    double Follow::Weight(const int i) const
    {
	return pow(i, 2); //pow(m_wbase, ((double) i) / (GetNrWaypts() - 1));
    }
    

    void Follow::Sample(const int i, double p[])
    {
	auto wpt = GetWaypt(i);
	auto coin = RandomUniformReal();

	if(coin < 0.7)
	{
	    p[0] = wpt[0];
	    p[1] = wpt[1];
	}	
	else if(coin < 0.85)
	     SampleRandomPointInsideCircle2D(wpt, m_tolReach, p);
	else if(coin < 0.95)
	    SampleRandomPointInsideConvexPolygon2D(4, &m_goals[8 * i], p);
	else
	    SampleRandomPointInsideCircle2D(wpt, wpt[2], p);
    }

    void Follow::ComputeBoundaries(const double width, const double dprev, const double dafter,  const double proot[])
    {
	auto n = GetNrWaypts();
	auto prev = proot;
	double v[2];
	double d;
	double p1[2];
	double p2[2];
	double u[2];

	m_quads.resize(8 * n);
	m_goals.resize(8 * n);
	
	for(int i = 0; i < n; ++i)
	{
	    auto p = GetWaypt(i);
	    v[0] = p[0] - prev[0];
	    v[1] = p[1] - prev[1];
	    d = sqrt(v[0] * v[0] + v[1] * v[1]);
	    v[0] /= d;
	    v[1] /= d;
	    u[0] = -width*v[1]; //u is perpendicular to v
	    u[1] = width*v[0];
	    
	    p1[0] = prev[0] - dprev * v[0];
	    p1[1] = prev[1] - dprev * v[1];
	    p2[0] = p[0] + dafter * v[0];
	    p2[1] = p[1] + dafter * v[1];

	    auto quad = &m_quads[8 * i];

	    quad[0] = p1[0] - u[0];  quad[1] = p1[1] - u[1];
	    quad[2] = p2[0] - u[0];  quad[3] = p2[1] - u[1];
	    quad[4] = p2[0] + u[0];  quad[5] = p2[1] + u[1];
	    quad[6] = p1[0] + u[0];  quad[7] = p1[1] + u[1];

	    auto goal = &m_goals[8 * i];
	    // m_tolReach = 1.5;
	    
	    goal[0] = p[0] - v[0] * m_tolReach - u[0]; goal[1] = p[1] - v[1] * m_tolReach - u[1];
	    goal[2] = p[0] + v[0] * m_tolReach - u[0]; goal[3] = p[1] + v[1] * m_tolReach - u[1];
	    goal[4] = p[0] + v[0] * m_tolReach + u[0]; goal[5] = p[1] + v[1] * m_tolReach + u[1];
	    goal[6] = p[0] - v[0] * m_tolReach + u[0]; goal[7] = p[1] - v[1] * m_tolReach + u[1];
	    
	    
	    //   printf("tolReach = %f\n", m_tolReach);
	    
	    
	    prev = p;
	}
	
    }
    
    void Follow::Draw(void)
    {
	const int n = GetNrWaypts();
	for(int i = 0; i < n - 1; ++i)
	{
	    const double *p1   = GetWaypt(i);
	    const double *p2   = GetWaypt(i + 1);
	    const double  d    = p1[2];
	    const double  vx   = p2[0] - p1[0];
	    const double  vy   = p2[1] - p1[1];
	    const double  norm = sqrt(vx * vx + vy * vy);
	    const double  ux   = -vy * d / norm;
	    const double  uy   =  vx * d/ norm;
	    
	    
	    GDrawSegment2D(p1, p2);
	    GDrawSegment2D(p1[0] + ux, p1[1] + uy, p2[0] + ux, p2[1] + uy);
	    GDrawSegment2D(p1[0] - ux, p1[1] - uy, p2[0] - ux, p2[1] - uy);
	    
	}

	GDrawWireframe(true);	
	for(int i = 0; i < n; ++i)
	    GDrawCircle2D(GetWaypt(i), GetWaypt(i)[2]);
	GDrawWireframe(false);

	for(int i = 0; i < n; ++i)
	    GDrawCircle2D(GetWaypt(i), m_tolReach);

	char msg[100];
	
	GDrawColor(0, 0, 0);
	for(int i = 0; i < n; ++i)
	{
	    sprintf(msg, "%d", i);
	    GDrawString2D(msg, GetWaypt(i)[0], GetWaypt(i)[1]);
	}
	
    }
    
}

