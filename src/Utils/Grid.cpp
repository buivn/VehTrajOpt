/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "Utils/Grid.hpp"
#include "Utils/Misc.hpp"
#include <cstdlib>

namespace GP
{
    void Grid::Setup(const int     ndims,
		     const int     dims[],
		     const double  min[],
		     const double  max[])
    {
	m_minUnit = INFINITY;
	
	m_ndims = ndims;	    
	m_dims.resize(ndims);
	m_min.resize(ndims);
	m_max.resize(ndims);
	m_units.resize(ndims);
	
	m_cvol   = 1;
	m_ncells = 1;
	for(int i = 0; i < ndims; ++i)
	{
	    m_dims[i] = dims[i];
	    m_min[i]  = min[i];
	    m_max[i]  = max[i];
	    m_units[i]= (m_max[i] - m_min[i]) / m_dims[i];
	    m_cvol   *= m_units[i];
	    m_ncells *= m_dims[i];

	    if(m_units[i] < m_minUnit)
		m_minUnit = m_units[i];
	    
	}
    }
    
    
    int Grid::GetCellIdFromCoords(const int coords[]) const
    {
	int  factor = 1;
	int  id     = 0;;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    id     += coords[i] * factor;
	    factor *= m_dims[i];
	}
	return id;
    }
    
    void Grid::GetCoordsFromCellId(const int id, int coords[]) const
    {
	int factor = id;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    coords[i] = factor % m_dims[i];
	    factor   /= m_dims[i];
	}
    }
    
    void Grid::GetCoords(const double p[], int coords[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	    coords[i] = GetCoord(p[i], m_min[i], m_max[i], m_units[i], m_dims[i]);	
    }
    
    int Grid::GetCellId(const double p[]) const
    {
	int factor = 1;
	int id     = 0;;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    id     += factor * GetCoord(p[i], m_min[i], m_max[i], m_units[i], m_dims[i]);
	    factor *= m_dims[i];
	}
	return id;	
    }
    
    void Grid::GetCellFromCoords(const int coords[], 
				 double    min[], 
				 double    max[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	{
	    min[i] = m_min[i] + m_units[i] * coords[i];
	    max[i] = min[i]   + m_units[i];
	}
    }
    
    
    void Grid::GetCellFromId(const int id, double min[], double max[]) const
    {
	int factor  = id;
	int coord_i = 0;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    coord_i   = factor % m_dims[i];
	    factor   /= m_dims[i];    
	    
	    min[i]    = m_min[i] + m_units[i] * coord_i;
	    max[i]    = min[i]   + m_units[i];
	}
	
    }
    
    void Grid::GetCellCenterFromCoords(const int coords[], double c[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	    c[i] =  m_min[i] + (0.5 + coords[i]) * m_units[i];

    }
    
    void Grid::GetCellCenterFromId(const int id, double c[]) const
    {
	int factor  = id;
	int coord_i = 0;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    coord_i   = factor % m_dims[i];
	    factor   /= m_dims[i];    

	    c[i] = m_min[i] + (0.5 + coord_i) * m_units[i];
	}
    }
    
	
    bool Grid::IsPointInside(const double p[]) const
    {
	for(int i = 0; i < m_ndims; ++i)
	    if(p[i] < m_min[i] || p[i] > m_max[i])
		return false;
	return true;	
    }
    
    bool Grid::IsPointInsideCell(const int coords[], const double p[]) const
    {
	double min;
	
	for(int i = 0; i < m_ndims; ++i)
	{
	    min = m_min[i] + m_units[i] * coords[i];
	    if(p[i] < min || p[i] > (min + m_units[i]))
		return false;
	}
	return true;
    }

    void Grid::GetNeighs(const int id, std::vector<int> *neighs) const
    {
	std::vector<int> coords;
	coords.resize(GetNrDims());
	GetCoordsFromCellId(id, &coords[0]);
	GetNeighs(id, &coords[0], neighs);
    }
    
    
    void Grid::GetNeighs(const int id, const int coords[], std::vector<int> *neighs) const
    {
	neighs->clear();

	const int neighs2d[] =
	    {
		-1, 0,
		+1, 0,
		0, -1,
		0, +1,
		-1, -1,
		1, -1,
		1, 1,
		-1, 1
	    };
	const int nrNeighs2d = 8;
		
	    
	const int neigh3d[] =
	    {
		-1,  0, 0,
		+1,  0, 0,
		0, -1, 0,
		0, +1, 0,
		-1, -1, 0,
		-1, +1, 0,
		+1, -1, 0,
		+1, +1, 0,
		
		-1,  0, -1,
		+1,  0, -1,
		0, -1, -1,
		0, +1, -1,
		-1, -1, -1,
		-1, +1, -1,
		+1, -1, -1,
		+1, +1, -1,
		
		-1,  0, +1,
		+1,  0, +1,
		0, -1, +1,
		0, +1, +1,
		-1, -1, +1,
		-1, +1, +1,
		+1, -1, +1,
		+1, +1, +1,
		
		0, 0, -1,
		0, 0, +1
	    };
 
	
	
	
	const int n      = m_dims.size();
	int       factor = 1;
	
	if(n == 2)
	{
	    
	    for(int i = 0; i < nrNeighs2d; ++i)
		if((coords[0] + neighs2d[2 * i + 0]) < m_dims[0] &&
		   (coords[0] + neighs2d[2 * i + 0])  >= 0       &&
		   (coords[1] + neighs2d[2 * i + 1]) < m_dims[1] &&
		   (coords[1] + neighs2d[2 * i + 1])  >= 0)
		    neighs->push_back(id + 
				      neighs2d[2 * i + 0] + 
				      neighs2d[2 * i + 1] * m_dims[0]);
	    
	   	
	        
	}
	else if(n == 3)
	{
	    for(int i = 0; i < 26; ++i)	    
		if((coords[0] + neigh3d[3 * i + 0]) < m_dims[0] &&
		   (coords[0] + neigh3d[3 * i + 0])  >= 0       &&
		   (coords[1] + neigh3d[3 * i + 1]) < m_dims[1] &&
		   (coords[1] + neigh3d[3 * i + 1])  >= 0       &&
		   (coords[2] + neigh3d[3 * i + 2]) < m_dims[2] &&
		   (coords[2] + neigh3d[3 * i + 2])  >= 0)
		    neighs->push_back(id + 
				      neigh3d[3 * i + 0] + 
				      neigh3d[3 * i + 1] * m_dims[0] +
				      neigh3d[3 * i + 2] * m_dims[0] * m_dims[1]);	
	}
    }

    
    void Grid::GetCellsInsideBox(const double p[], const double d, std::vector<int> &cids)
    {
	auto nrDims = GetNrDims();
	auto id = GetCellId(p);


	std::vector<int> coords(nrDims);
   	std::vector<int> coordsMax(nrDims);
	std::vector<int> coordsMin(nrDims);
	std::vector<int> counter(nrDims);
	std::vector<int> counterLimits(nrDims);	
		
	GetCoordsFromCellId(id, &coords[0]);	
	for(int i = nrDims - 1; i >= 0; --i)
	{
	    coordsMax[i] = GetCoord(p[i] + d, GetMin()[i], GetMax()[i], GetUnits()[i], GetDims()[i]);
	    coordsMin[i] = GetCoord(p[i] - d, GetMin()[i], GetMax()[i], GetUnits()[i], GetDims()[i]);
	    counterLimits[i] = coordsMax[i] - coordsMin[i] + 1;
	    counter[i] = 0;
	}

	cids.clear();
	do
	{
	    for(int i = nrDims - 1; i >= 0; --i)
		coords[i] = coordsMin[i] + counter[i];
	    cids.push_back(GetCellIdFromCoords(&coords[0]));
	}
	while(NextCounter(nrDims, &counter[0], &counterLimits[0]));
	
    }

    void Grid::GetCellsInsideBall(const double p[], const double r, std::vector<int> &cids)
    {
	auto nrDims = GetNrDims();
	
	std::vector<double> c(nrDims);
	
	GetCellsInsideBox(p, r, cids);
	for(int i = 0; i < cids.size();)
	{
	    GetCellCenterFromId(cids[i], &c[0]);
	    if(Algebra::PointDistSquared(nrDims, p, &c[0]) > r * r)
	    {
		cids[i] = cids.back();
		cids.pop_back();
	    }
	    else
		++i;	    
	}	
    }

    
}
