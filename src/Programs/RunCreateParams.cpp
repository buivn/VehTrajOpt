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

extern "C" int RunCreateParams(int argc, char **argv)
{
    GP::ScenePolys sceneKnownObstacles;	
    GP::Params params;
    char nameSceneType[300];
    
    printf("usage: ./bin/Runner RunCreateParams fnameParams\n");
    
    if(argc < 1)
	return 0;
       
    FILE *in = fopen(argv[1], "r");
    params.Read(in);
    fclose(in);
    params.ParseArgs(2, argc-1, argv);
    params.Print(stdout);


    sceneKnownObstacles.SetupFromParams(&params);
    auto fname = params.GetValue("SceneOccupancyFile", NULL);
    if(fname)
    {
	sprintf(nameSceneType, "SceneOccupancyFile %s", fname);	
	sceneKnownObstacles.ReadOccupancyGrid(fname);
    }    
    else if((fname = params.GetValue("SceneObstaclesFile", NULL)))
    {
	sprintf(nameSceneType, "SceneObstaclesFile %s", fname);	
	sceneKnownObstacles.ReadObstacles(fname);	
    }
    
    sceneKnownObstacles.AddBoundary(params.GetValueAsDouble("SceneBoundaryThickness", GP::Constants::SCENE_BOUNDARY_THICKNESS));

    auto  qinitMinX = params.GetValueAsDouble("QueryInitMinX", -40);
    auto  qinitMaxX = params.GetValueAsDouble("QueryInitMaxX",  40);
    auto  qinitMinY = params.GetValueAsDouble("QueryInitMinY", -40);
    auto  qinitMaxY = params.GetValueAsDouble("QueryInitMaxY", -30);
    auto qinitMaxLinkLength = params.GetValueAsDouble("QueryInitMaxLinkLength", 2);
    auto qinitMaxNrLinks = params.GetValueAsInt("QueryInitMaxNrLinks", 10);
    auto qinitMaxWidth = params.GetValueAsDouble("QueryInitMaxWidth", 3);
    
    auto  qgoalMinX = params.GetValueAsDouble("QueryGoalMinX", -40);
    auto  qgoalMaxX = params.GetValueAsDouble("QueryGoalMaxX",  40);
    auto  qgoalMinY = params.GetValueAsDouble("QueryGoalMinY",  30);
    auto  qgoalMaxY = params.GetValueAsDouble("QueryGoalMaxY",  40);
    auto  qgoalMaxRadius = params.GetValueAsDouble("QueryGoalMaxRadius",  4);

    GP::Polygon2D pstart;
    double sx = 0;
    double sy = 0;
    
    do
    {
	sx = GP::RandomUniformReal(qinitMinX,  qinitMaxX);
	sy = GP::RandomUniformReal(qinitMinY, qinitMaxY);	
	auto minx = sx - qinitMaxNrLinks * qinitMaxLinkLength;
	auto maxx = sx + qinitMaxLinkLength;
	auto miny = sy - 0.5 * qinitMaxWidth;
	auto maxy = sy + 0.5 * qinitMaxWidth;

	pstart.Clear();
	pstart.AddVertex(minx, miny);
	pstart.AddVertex(maxx, miny);
	pstart.AddVertex(maxx, maxy);
	pstart.AddVertex(minx, maxy);
    }
    while(sceneKnownObstacles.IsInCollision(pstart));


    GP::Polygon2D pgoal;
    const int nrPtsForCircle = 5;
    std::vector<double> pts(2 * nrPtsForCircle);
    double gx = 0;
    double gy = 0;
    
    do
    {
	gx = GP::RandomUniformReal(qgoalMinX,  qgoalMaxX);
	gy = GP::RandomUniformReal(qgoalMinY, qgoalMaxY);

	GP::CircleAsPolygon2D(gx, gy, qgoalMaxRadius, nrPtsForCircle, &pts[0]);


	pgoal.Clear();
	pgoal.AddVertices(nrPtsForCircle, &pts[0]);	
    }
    while(sceneKnownObstacles.IsInCollision(pgoal));
    

   fname = params.GetValue("ParamsOut");
    if(fname)
    {
	auto out = fopen(fname, "w");
	fprintf(out, "%s\n",  nameSceneType);	
	fprintf(out, "SnakeStartX %f\n", sx);
	fprintf(out, "SnakeStartY %f\n", sy);
	fprintf(out, "GoalX %f\n", gx);
	fprintf(out, "GoalY %f\n", gy);
	fprintf(out, "ParamsExtraFile data/ParamsGeneral%s.txt\n", params.GetValue("SceneName", ""));
	fclose(out);
	
    }
    
    return 0;
}


extern "C" int RunCreateParamsScene10(int argc, char **argv)
{
    char cmd[300];
    
    for(int nrLines = 5; nrLines <= 10; ++nrLines)
    {
	for(int i = 0; i < 30; ++i)
	{
	    sprintf(cmd, "./bin/Runner RunCreateParams data/ParamsScene10.txt SceneObstaclesFile data/scene10/Scene10_Lines%d_%dObstacles.txt ParamsExtraFile data/scene10/Scene10_Lines%d_%dQueryBounds.txt ParamsOut data/scene10/ParamsScene10_Lines%d_%d.txt SceneName Scene10", nrLines, i, nrLines, i, nrLines, i);

	    system(cmd);
	    
	}
	
    }

    return 0;
   
}

extern "C" int RunCreateParamsScene11(int argc, char **argv)
{
    char cmd[300];
    
    for(int nrLines = 12; nrLines <= 20; ++nrLines)
    {
	for(int i = 0; i < 30; ++i)
	{
	    sprintf(cmd, "./bin/Runner RunCreateParams data/ParamsScene11.txt SceneObstaclesFile data/scene11/Scene11_Frac%d_%dObstacles.txt ParamsExtraFile data/scene11/Scene11_Frac%d_%dQueryBounds.txt ParamsOut data/scene11/ParamsScene11_Frac%d_%d.txt  SceneName Scene11", nrLines, i, nrLines, i, nrLines, i);

	    system(cmd);
	    
	}
	
    }

    return 0;
    
    
}

extern "C" int RunCreateParamsScene12(int argc, char **argv)
{
    char cmd[300];
    
    for(int nrLines = 10; nrLines <= 18; ++nrLines)
    {
	for(int i = 0; i < 30; ++i)
	{
	    sprintf(cmd, "./bin/Runner RunCreateParams data/ParamsScene12.txt SceneObstaclesFile data/scene12/Scene12_Dims%d_%dObstacles.txt ParamsExtraFile data/scene12/Scene12_Dims%d_%dQueryBounds.txt ParamsOut data/scene12/ParamsScene12_Dims%d_%d.txt  SceneName Scene12", nrLines, i, nrLines, i, nrLines, i);

	    system(cmd);
	    
	}
	
    }

    return 0;
    
    
}

extern "C" int RunCreateParamsSceneRings(int argc, char **argv)
{
    char cmd[300];
    
    for(int nrLines = 2; nrLines <= 15; ++nrLines)
    {
	for(int i = 0; i < 30; ++i)
	{
	    sprintf(cmd, "./bin/Runner RunCreateParams data/ParamsSceneRings.txt SceneObstaclesFile data/sceneRings/SceneRings_Gap%d_%dObstacles.txt ParamsExtraFile data/sceneRings/SceneRings_Gap%d_%dQueryBounds.txt ParamsOut data/sceneRings/ParamsSceneRings_Gap%d_%d.txt  SceneName SceneRings", nrLines, i, nrLines, i, nrLines, i);

	    system(cmd);
	    
	}
	
    }

    return 0;
    
    
}


