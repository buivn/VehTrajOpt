/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "TrajOpt/DevPlanner.hpp"
#include "GP/Goal.hpp"
#include "GP/SceneCells.hpp"
#include "GP/GridDecomposition.hpp"
#include "GP/FollowPlanner.hpp"
#include "GP/MPSnakeSimulator.hpp"


namespace GP
{
	void TrajOptPlanner::Plan(const PlannerInput &pin, Solution &sol) {	    
	    m_decomp.CreateRegions();	    
	    m_decomp.ReadyForMotionPlanning();
	    auto res = IsStartConnectedToGoalViaGridPath();
	    if(res == false) {
			sol.Clear();
			sol.m_solutionFound = false;
			printf("...not running motion planner since start & goal are not connected via grid path\n");
	    }	    
	    else  {
			sol.m_solutionFound = true;		
			InitializeAndRunPlanner(pin, sol);
	    }
	    sol.m_goalX = pin.m_goalX;
	    sol.m_goalY = pin.m_goalY;
	    sol.m_goalRadius = pin.m_goalRadius;
	    
	}

	void TrajOptPlanner::SnakePlacement(const PlannerInput &pin, const double s[], double pts[])
	{
		std::vector<double> bodies;
		double   orig[8];
		double TR[GP::Algebra2D::TransRot_NR_ENTRIES];
		double T[GP::Algebra2D::Trans_NR_ENTRIES];
		double p[2];
		
		const double hw = 0.5 * pin.m_snakeLinkWidth;
		
		orig[0] = 0;
		orig[1] = -hw;
		orig[2] = pin.m_snakeLinkLength;
		orig[3] = -hw;	
		orig[4] = pin.m_snakeLinkLength;
		orig[5] =  hw;	
		orig[6] = 0;
		orig[7] =  hw;	

		TR[0] = p[0] = s[GP::MPSnakeSimulator::STATE_X];
		TR[1] = p[1] = s[GP::MPSnakeSimulator::STATE_Y];
		GP::Algebra2D::AngleAsRot(s[GP::MPSnakeSimulator::STATE_THETA], &TR[GP::Algebra2D::Trans_NR_ENTRIES]);
		GP::ApplyTransRotToPolygon2D(TR, 4, orig, &pts[0]);
			
		for(int i = 0; i < pin.m_snakeNrLinks; ++i)
		{
		T[0] = p[0] - (pin.m_snakeAttachDistance + pin.m_snakeLinkLength);
		T[1] = p[1];
		GP::Algebra2D::RotateAroundPointAsTransRot(s[GP::MPSnakeSimulator::STATE_LINKS + i], p, TR);
		GP::Algebra2D::TransRotMultTransAsTransRot(TR, T, TR);
		GP::ApplyTransRotToPolygon2D(TR, 4, orig, &pts[8 * (i + 1)]);
		p[0] = TR[0];
		p[1] = TR[1];
		}
	}



	void TrajOptPlanner::RunMotionPlannerStartToGoal(PlannerInput &pin,  Solution &sol) {
		sol.Clear();		
		Setup(pin);
		Plan(pin, sol);
		m_decomp.ExportRegions(pin.m_decompRegions);
		
		sol.PrintInfo();
	}

	// void TrajOptPlanner::RunMotionPlannerPointToGoal(PlannerInput &pin,  Solution &sol)
	// {
	// 	sol.Clear();		
	// 	const int nrTries = 100000000;
	// 	SetupScene(pin);
	// 	SetupSimulator(pin);
	// 	SetupDecomposition(pin);    
	// 	SetupOccupancyGrid(pin);
	// 	SetupGoal(pin);
	// 	auto res  = m_sim.SampleValidStateNearPos(pin.m_startState,  pin.m_fromPointToStateSampleRadius,  pin.m_fromPointToStateSampleScaleAngle, pin.m_fromPointToStateSampleNrAttempts);
	// 	if(res == false)
	// 	{
	// 	sol.m_solutionFound = false;
	// 	printf("RunMotionPlannerPointToGoal...failed since it could not generate a valid start state\n");	
	// 	return;
	// 	}
		
		
	// 	SetupStartState(pin);
	// 	SetupPlanner(pin);
		
	// 	Plan(pin, sol);
	// 	sol.PrintInfo();
	// }
	
	

}

