/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__HEIGHT_FIELD_HPP_
#define GP__HEIGHT_FIELD_HPP_

#include "Utils/Grid.hpp"
#include <cstdlib>

namespace GP
{

    class HeightField
    {
    public:
	HeightField(void)
	{
	}
	
	virtual ~HeightField(void)
	{
	}

	virtual const Grid* GetGrid(void) const
	{
	    return &m_grid;
	}

	virtual Grid* GetGrid(void)
	{
	    return &m_grid;
	}
	
	virtual double GetHeightAtCell(const int id) const
	{
	    return m_heights[id];
	}
	
	virtual double GetHeightAtCell(const int i, const int j) const
	{
	    const int coords[2] = {i, j};
	    return GetHeightAtCell(m_grid.GetCellIdFromCoords(coords));
	}
	
	virtual double GetHeightAtPoint(const double x, const double y) const
	{
	    const double p[2] = {x, y};
	    return GetHeightAtCell(m_grid.GetCellId(p));
	    
	}

	virtual void CompleteSetup(void)
	{
	    m_heights.resize(m_grid.GetNrCells());
	}
	

	virtual void SetHeightAtCell(const int id, const double h)
	{	   
	    m_heights[id] = h;
	    
	}

    protected:	
	Grid                m_grid;
	std::vector<double> m_heights;
    };

   
}

#endif





    
    

