#ifndef GP__SETUP_HPP_
#define GP__SETUP_HPP_

#include "TrajOpt/DevPlanner.hpp"
#include "Utils/Params.hpp"
#include "GP/FollowPlanner.hpp"

namespace GP
{
    class Setup
    {
    public:
	Setup(void)
	{
	}

	virtual ~Setup(void)
	{
	}

	static void SetupPlannerInput(Params &params, PlannerInput &pin)
	{
	    pin.m_fromPointToStateSampleRadius = params.GetValueAsDouble("FromPointToStateSampleRadius", pin.m_fromPointToStateSampleRadius);
	    pin.m_fromPointToStateSampleNrAttempts = params.GetValueAsInt("FromPointToStateSampleNrAttempts", pin.m_fromPointToStateSampleNrAttempts);
	    pin.m_fromPointToStateSampleScaleAngle = params.GetValueAsDouble("FromPointToStateSampleScaleAngle", pin.m_fromPointToStateSampleScaleAngle);
	     
	    pin.m_gridNrDimsX = params.GetValueAsInt("SceneGridNrDimsX", pin.m_gridNrDimsX);
	    pin.m_gridNrDimsY = params.GetValueAsInt("SceneGridNrDimsY", pin.m_gridNrDimsY);
 	    pin.m_gridMinX = params.GetValueAsDouble("SceneGridMinX", pin.m_gridMinX);
  	    pin.m_gridMinY = params.GetValueAsDouble("SceneGridMinY", pin.m_gridMinY);
	    pin.m_gridMaxX = params.GetValueAsDouble("SceneGridMaxX", pin.m_gridMaxX);
  	    pin.m_gridMaxY = params.GetValueAsDouble("SceneGridMaxY", pin.m_gridMaxY);
	    
	    pin.m_snakeNrLinks = params.GetValueAsInt("SnakeNrLinks", pin.m_snakeNrLinks);
	    pin.m_snakeLinkLength = params.GetValueAsDouble("SnakeLinkLength", pin.m_snakeLinkLength);
	    pin.m_snakeLinkWidth = params.GetValueAsDouble("SnakeLinkWidth", pin.m_snakeLinkWidth);
	    pin.m_snakeAttachDistance = params.GetValueAsDouble("SnakeAttachDistance", pin.m_snakeAttachDistance);
	    
	    pin.m_goalX = params.GetValueAsDouble("GoalX", pin.m_goalX);
	    pin.m_goalY = params.GetValueAsDouble("GoalY", pin.m_goalY);
	    pin.m_goalRadius = params.GetValueAsDouble("GoalRadius", pin.m_goalRadius);
	    pin.m_goalNrSides = params.GetValueAsInt("GoalNrSides", pin.m_goalNrSides);

	    pin.m_tmax = params.GetValueAsDouble("PlannerMaxRuntime", pin.m_tmax);
	    pin.m_plannerGuideSeparationDistance = params.GetValueAsDouble("PlannerGuideSeparationDistance", pin.m_plannerGuideSeparationDistance);
	    pin.m_plannerGuideWidth = params.GetValueAsDouble("PlannerGuideWidth", pin.m_plannerGuideWidth);
	    pin.m_plannerGuideBorderExtendBefore = params.GetValueAsDouble("PlannerGuideBorderExtendBefore", pin.m_plannerGuideBorderExtendBefore);
	    pin.m_plannerGuideBorderExtendAfter = params.GetValueAsDouble("PlannerGuideBorderExtendAfter", pin.m_plannerGuideBorderExtendAfter);
	    pin.m_plannerGuideReachDistance = params.GetValueAsDouble("PlannerGuideReachDistance", pin.m_plannerGuideReachDistance);
	    pin.m_plannerClearExponent = params.GetValueAsDouble("PlannerClearExponent", pin.m_plannerClearExponent);
	    pin.m_plannerClearMax = params.GetValueAsDouble("PlannerClearMax", pin.m_plannerClearMax);
 	    pin.m_plannerScaleRadix = (long) params.GetValueAsDouble("PlannerScaleRadix", pin.m_plannerScaleRadix);
  	    pin.m_plannerUseRadix = params.GetValueAsInt("PlannerUseRadix", pin.m_plannerUseRadix);
  
	    pin.m_plannerRegionsNrDimsX = params.GetValueAsInt("PlannerRegionsNrDimsX", pin.m_plannerRegionsNrDimsX);
	    pin.m_plannerRegionsNrDimsY = params.GetValueAsInt("PlannerRegionsNrDimsY", pin.m_plannerRegionsNrDimsY);

	    pin.m_plannerMaxCountExploreAbandon = params.GetValueAsInt("PlannerMaxCountExploreAbandon", pin.m_plannerMaxCountExploreAbandon);	    
	    pin.m_plannerMaxCountFollowRetry = params.GetValueAsInt("PlannerMaxCountFollowRetry", pin.m_plannerMaxCountFollowRetry);
	    pin.m_plannerMaxCountFollowAbandon = params.GetValueAsInt("PlannerMaxCountFollowAbandon", pin.m_plannerMaxCountFollowAbandon);
	
	    auto dimState = pin.m_snakeNrLinks + 5;	    
	    pin.m_startState = new double[dimState];
	    std::fill(pin.m_startState, pin.m_startState + dimState, 0.0);
	    pin.m_startState[0] = params.GetValueAsDouble("SnakeStartX", pin.m_startState[0]);
	    pin.m_startState[1] = params.GetValueAsDouble("SnakeStartY", pin.m_startState[1]);
	    pin.m_startState[2] = params.GetValueAsDouble("SnakeStartTheta", pin.m_startState[2]);
 	    pin.m_startState[3] = params.GetValueAsDouble("SnakeStartVeleocity", pin.m_startState[3]);
 	    pin.m_startState[4] = params.GetValueAsDouble("SnakeStartSteer", pin.m_startState[4]);
	    char msg[300];
	    for(int i = 0; i < pin.m_snakeNrLinks; ++i)
	    {
		sprintf(msg, "SnakeStartTrailer%dTheta", i);		
		pin.m_startState[5 + i] = params.GetValueAsDouble(msg, pin.m_startState[5 + i]);
	    }
		    
	    pin.m_occupancy = new int[pin.m_gridNrDimsX * pin.m_gridNrDimsY];

	    auto method = params.GetValue("PlannerRunAsMethod", NULL);
	    if(method == NULL)
		pin.m_plannerRunAsMethod = FollowPlanner::RUN_AS_NORMAL;
	    else if(strcmp(method, "RRT") == 0)
		pin.m_plannerRunAsMethod = FollowPlanner::RUN_AS_RRT;
	    else if(strcmp(method, "GUST") == 0)
		pin.m_plannerRunAsMethod = FollowPlanner::RUN_AS_GUST;
	    else if(strcmp(method, "EST") == 0)
		pin.m_plannerRunAsMethod = FollowPlanner::RUN_AS_EST;
	    else
		pin.m_plannerRunAsMethod = FollowPlanner::RUN_AS_NORMAL;
	    
	    pin.m_plannerMinClearAcceptHint = params.GetValueAsDouble("PlannerMinClearAcceptHint", pin.m_plannerMinClearAcceptHint);

	}
    };
    
    
}

#endif



    







