/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__GPCONSTANTS_HPP_
#define GP__GPCONSTANTS_HPP_

#include "Utils/Constants.hpp"
#include <cmath>

namespace GP
{
    namespace Constants
    {

//Sensor
	const char KW_Sensor[] = "Sensor";	
	const char KW_Radius[] = "Radius";
	const double SENSOR_RADIUS = 100.0;
		
	const int    MEDIAL_AXIS_GRID_DIMX = 512;
	const int    MEDIAL_AXIS_GRID_DIMY = 512;
	const int    SCENE_GRID_DIMX = 64;
	const int    SCENE_GRID_DIMY = 64;
	const double SCENE_GRID_MINX = -40.0;
	const double SCENE_GRID_MINY = -40.0;
	const double SCENE_GRID_MAXX =  40.0;
	const double SCENE_GRID_MAXY =  40.0;
	const double SCENE_OBSTACLES_HEIGHT = 2.0;
	const double SCENE_BOUNDARY_HEIGHT  = 3.0;
	const double SCENE_BOUNDARY_THICKNESS = 0.4;
	
	const double SIMULATOR_TIME_STEP             = 0.15;
	const double SIMULATOR_MIN_DISTANCE_ONE_STEP = 0.7;
	const double SIMULATOR_MAX_DISTANCE_ONE_STEP = 1.0;

	const bool   ROBOT_FRONT_WHEEL_DRIVING = true;
	const double ROBOT_BODY_LENGTH         = 1.75;
	const double ROBOT_BODY_WIDTH          = 1.0;
	const double ROBOT_MIN_STEER_ANGLE     = -88 * Constants::DEG2RAD;
	const double ROBOT_MAX_STEER_ANGLE     =  88 * Constants::DEG2RAD;
	const double ROBOT_MIN_VELOCITY        = -2.0;
	const double ROBOT_MAX_VELOCITY        =  2.0;
	const double ROBOT_MIN_ACCELERATION    = -2;
	const double ROBOT_MAX_ACCELERATION    =  2;
	const double ROBOT_MIN_STEER_VELOCITY  = -3;
	const double ROBOT_MAX_STEER_VELOCITY  = 3;
	const double SNAKE_NR_LINKS            = 5;
	const double SNAKE_ATTACH_DISTANCE     = 0.01;
	
	const double PLANNER_SELECTION_PENALTY                 = 0.95;
	const double PLANNER_PROBABILITY_FOLLOW_PATH           = 0.9;
	const double PLANNER_PROBABILITY_SELECT_NEAREST_VERTEX = 0.2;
	const double PLANNER_EDGE_COST_INCREASE_FRACTION       = 0.025;
	const int    PLANNER_WHEN_TO_RECOMPUTE_PATHS_TO_GOAL   = 2000;	
	const double PLANNER_GOAL_BIAS                         = 0.05;
	const double PLANNER_TOLERANCE_STEER                   = 2.0;
	const double PLANNER_PROBABILITY_STEER                 = 1.0;
	const int    PLANNER_EXTEND_MIN_NR_STEPS               = 400;
	const int    PLANNER_EXTEND_MAX_NR_STEPS               = 600;


	const double DECOMPOSITION_EXPONENT_CLEARANCE = 6;
	const double DECOMPOSITION_EXPONENT_COST_PATH_TO_GOAL = 4 ;
	const double DECOMPOSITION_MIN_TRIANGLE_AREA = 0.025;
	const double DECOMPOSITION_AVG_TRIANGLE_AREA = 1.00;
	const double DECOMPOSITION_GEMAN_MCLURE_SIGMA= 2.00;
	
	
	const double DRAW_ZORDER_GOAL          = 0.001;
	const double DRAW_ZORDER_ROBOT         = 0.001;
	const double DRAW_ZORDER_PLANNER       = 0.001;
	const double DRAW_ZORDER_REGION_CENTER = 0.002;

	const double FOLLOW_WEIGHT_BASE     = 100000000.0;
	const double FOLLOW_REACH_TOLERANCE = 2.0;
	const double FOLLOW_RADIUS          = 3.0;
	

	const int MAX_NR_QUERIES = 2;
	const int NR_RUNS_PER_QUERY = 3;
	const double WAIT_LOCK_TIME = 10.0;
	
    }
}

#endif
    
    
    
    







