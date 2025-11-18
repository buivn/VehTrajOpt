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

#ifndef GP__Sensor_HPP_
#define GP__Sensor_HPP_

#include "GP/ScenePolys.hpp"
#include "GP/GPConstants.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace GP
{

    class Sensor 
    {
    public:
	Sensor( void ) :
	    m_radius(Constants::SENSOR_RADIUS),
	    m_scene(NULL)
	{
	}
	
	virtual ~Sensor( void )
	{
	    Clear();	    
	}

	struct Data
	{
	    Data(void) : m_hasNew(true)
	    {
	    }
	    
	    std::unordered_set<int> m_cellsFree;
	    std::unordered_set<int> m_cellsOccupied;
	    bool m_hasNew;
	    
	};
	
	    

	virtual double GetRadius(void) const
	{
	    return m_radius;
	}

	virtual void SetRadius(const double r)
	{
	    m_radius = r;
	}
	
	virtual void SetupFromParams( Params &params )
	{
	    SetRadius(params.GetValueAsDouble(Constants::KW_Radius, GetRadius()));
	}

	virtual void Clear(void);
		
	virtual const Data* Sense(const double p[]);

	virtual const std::unordered_set<int>* GetSensedFreeCells(void) const
	{
	    return &m_sensedCellsFree;
	}
	
	virtual std::unordered_set<int>* GetSensedFreeCells(void) 
	{
	    return &m_sensedCellsFree;
	}

	virtual const std::unordered_set<int>* GetSensedOccupiedCells(void) const
	{
	    return &m_sensedCellsOccupied;
	}
	
	virtual std::unordered_set<int>* GetSensedOccupiedCells(void) 
	{
	    return &m_sensedCellsOccupied;
	}


	ScenePolys *m_scene;
	
    protected:
	
	double m_radius;	
	std::unordered_set<int> m_sensedCellsFree;
	std::unordered_set<int> m_sensedCellsOccupied;
	std::unordered_map<int, Data*> m_sensorReadings;

    };

}

#endif
