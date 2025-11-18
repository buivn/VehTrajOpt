/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */
#include "GP/GPConstants.hpp"
#include "GP/ScenePolys.hpp"
#include "TrajOpt/DevPlanner.hpp"
#include "Programs/Archives.hpp"
#include "GP/Sensor.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Stats.hpp"
#include "Utils/Algebra2D.hpp"
#include <fstream>
#include "GP/Setup.hpp"
#include "Utils/Timer.hpp"

extern "C" int RunKnown(int argc, char **argv)
{
    GP::ScenePolys sceneKnownObstacles;	
    GP::Params params;
    GP::PlannerInput pin;    
    GP::Solution solution;
    
    GP::TrajOptPlanner planner;
    
    printf("usage: ./bin/Runner RunKnown fnameParams\n");
    
    if(argc < 1) return 0;
       
    FILE *in = fopen(argv[1], "r");
    params.Read(in);
    fclose(in);
    params.ParseArgs(2, argc-1, argv);
    // params.Print(stdout);

    
    GP::Setup::SetupPlannerInput(params, pin);
    bool saved_traj = true;
    
    sceneKnownObstacles.SetupFromParams(&params);
    auto fname = params.GetValue("SceneOccupancyFile", NULL);
    if(fname)
	sceneKnownObstacles.ReadOccupancyGrid(fname);
    else if((fname = params.GetValue("SceneObstaclesFile", NULL)))
	sceneKnownObstacles.ReadObstacles(fname);
    sceneKnownObstacles.AddBoundary(params.GetValueAsDouble("SceneBoundaryThickness", GP::Constants::SCENE_BOUNDARY_THICKNESS));


 //occupancy grid
    
    pin.m_tmax = params.GetValueAsDouble("RunKnownMaxRuntime", 60);
    auto runKnownStatsFile = params.GetValue("RunKnownStatsFile", "data/results/stats.txt");
    
    GP::Timer::Clock clk;
    GP::Timer::Clock clkRun;

    GP::Timer::Start(&clkRun);
    

    std::fill(pin.m_occupancy, pin.m_occupancy + pin.m_gridNrDimsX * pin.m_gridNrDimsY, GP::GRID_CELL_UNKNOWN);
    auto cids = &(sceneKnownObstacles.m_cidsObstacles);
    for(auto & cid : *cids)
	pin.m_occupancy[cid] = GP::GRID_CELL_OBSTACLE;
    cids = &(sceneKnownObstacles.m_cidsFree);
    for(auto & cid : *cids)
	pin.m_occupancy[cid] = GP::GRID_CELL_FREE;	  
    
    pin.m_statesRemainingInCurrentSolution.clear();
    
    GP::Timer::Start(&clk);

    // planner.Setup(pin);
    
    planner.RunMotionPlannerStartToGoal(pin, solution);
    GP::Stats::GetSingleton()->AddValue("TimeAll", GP::Timer::Elapsed(&clk));
    
    GP::Stats::GetSingleton()->PrintValues(stdout);
 
    const double goal[] = {pin.m_goalX, pin.m_goalY};
    bool solved = false;
    
    for(int i = 0; i < solution.TrajGetNrStates(); ++i)
    {
	auto scurr = solution.TrajGetState(i);
	if(GP::Algebra2D::PointDist(scurr, goal) <= pin.m_goalRadius)
	    solved = true;
	
    }
    if (saved_traj) {
        GP::Archives archives;
        archives.SaveTrajectory(solution);
    }    
    
    printf("Overall Distance Traveled = %f [solved = %d]\n", solution.m_trajCostOverall, solved);
    
    auto out = std::ofstream(runKnownStatsFile);
    out
	<< solved << " "
	<< GP::Stats::GetSingleton()->GetValue("TimeAll") << " "
	<< solution.m_trajCostOverall << " "
	<< pin.m_plannerRunAsMethod << std::endl;    
    out.close();
    
    return 0;
}


