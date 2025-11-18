/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__GRID_DECOMPOSITION_HPP_
#define GP__GRID_DECOMPOSITION_HPP_

#include "GP/Decomposition.hpp"
#include "GP/GridRegion.hpp"

namespace GP
{
    class GridDecomposition : public Decomposition
    {
    public:
	GridDecomposition() : Decomposition()
	{
	}

	virtual ~GridDecomposition()
	{
	}

	virtual void ExportRegions(std::vector<double> &boxes) ;

	virtual void CreateRegions(void);
	

	virtual int  LocateRegion(const double p[], const bool  allowInsideObstacleCell = false);


	virtual void ComputeGoalConnections(void);

	Grid m_grid;	
    };
}

#endif	


