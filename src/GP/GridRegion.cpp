/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/GridRegion.hpp"
#include "Utils/Geometry.hpp"

namespace GP
{
    
    void GridRegion::DrawBoundary(Region * r)
    {
	GridRegion *gr = dynamic_cast<GridRegion*>(r);
	if(gr == NULL)
	    return;
	int common[2];
	int count = 0;

	double poly1[8];
	double poly2[2];
	
	AABoxAsPolygon2D(&m_bbox[0], &m_bbox[2], poly1);
	AABoxAsPolygon2D(&(gr->m_bbox[0]), &(gr->m_bbox[2]), poly2);
	
	
	for(int i = 0; i < 4 && count < 2; ++i)
	    if(Algebra2D::PointDist(&poly1[2*i], &poly2[0]) <= Constants::EPSILON ||
	       Algebra2D::PointDist(&poly1[2*i], &poly2[2]) <= Constants::EPSILON ||
	       Algebra2D::PointDist(&poly1[2*i], &poly2[4]) <= Constants::EPSILON ||
	       Algebra2D::PointDist(&poly1[2*i], &poly2[6]) <= Constants::EPSILON)
	     	common[count++] = i;
	if(count >= 2)
	    GDrawSegment2D(&poly1[common[0]], &poly1[common[1]]);
    }	
}
