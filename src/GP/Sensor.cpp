/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


/*
 * Copyright (C) 2020 Erion Plaku
 * All Rights Reserved
 *
 *   Created by Erion Plaku
 *   www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#include "GP/Sensor.hpp"

namespace GP
{
    void Sensor::Clear(void)
    {
	m_sensedCellsFree.clear();
	m_sensedCellsOccupied.clear();

	for(auto & iter : m_sensorReadings)
	    delete iter.second;	
	m_sensorReadings.clear();
	
    }
    
    const Sensor::Data* Sensor::Sense(const double p1[])
    {
	std::vector<int> candidates;
	auto grid = m_scene->GetGrid();	     
	auto cid1 = grid->GetCellId(p1);
	std::vector<double> p2(grid->GetNrDims());
	std::vector<double> p3(grid->GetNrDims());

	auto iter = m_sensorReadings.find(cid1);
	if(iter != m_sensorReadings.end())
	{
	    iter->second->m_hasNew = false;
	    return iter->second;	
	}
	
	auto data = new Data();
	data->m_hasNew = false;
	
	m_sensorReadings.insert(std::make_pair(cid1, data));
	
	grid->GetCellsInsideBall(p1, GetRadius(), candidates);
	for(auto & cid2 : candidates)
	{
	    grid->GetCellCenterFromId(cid2, &p2[0]);
	    
	    auto d = Algebra::PointDist(grid->GetNrDims(), p1, &p2[0]);	
	    auto step = grid->GetMinUnit();
	    auto nrSteps = 10 + std::ceil(d / step);
	    
	    for(int i = 0; i <= nrSteps; ++i)
	    {
		auto t = (i + 0.0) / nrSteps;	    
		Algebra::VecLinearInterpolation(grid->GetNrDims(), p1, (1 - t), &p2[0], t, &p3[0]);
		auto cid3 = grid->GetCellId(&p3[0]);		     
		if(m_scene->IsCellInCollision(cid3) )
		{
		    data->m_cellsOccupied.insert(cid3);
		    auto res = m_sensedCellsOccupied.insert(cid3);
		    if(res.second == true) //insertion took place
			data->m_hasNew = true;
		    break;
		}
		else
		{
		    data->m_cellsFree.insert(cid3);
		    auto res = m_sensedCellsFree.insert(cid3);
		    //if(res.second == true) //insertion took place
		    //data->m_hasNew = true;
		    
		}
	    }
	}

	return data;	    
    }

}
