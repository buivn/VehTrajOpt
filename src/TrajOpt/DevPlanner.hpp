/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__DEV_PLANNER_HPP_
#define GP__DEV_PLANNER_HPP_

#include "GP/Solution.hpp"
#include "Programs/Archives.hpp"
#include "GP/Goal.hpp"
#include "GP/SceneCells.hpp"
#include "GP/GridDecomposition.hpp"
#include "GP/FollowPlanner.hpp"
#include "GP/MPSnakeSimulator.hpp"
#include "Utils/Params.hpp"
#include "GP/Sensor.hpp"
#include <map>

namespace GP
{
const int GRID_CELL_OBSTACLE = 1;
const int GRID_CELL_FREE = 0;
const int GRID_CELL_UNKNOWN = -1;

struct PlannerInput
{
PlannerInput(void)
{
	
	m_gridNrDimsX = 64;
	m_gridNrDimsY = 64;    
	m_gridMinX = -40.0;
	m_gridMinY = -40.0;
	m_gridMaxX = 40.0;
	m_gridMaxY = 40.0;
	m_occupancy = NULL;
	
	m_snakeNrLinks = 5;
	m_snakeLinkLength = 0.7;
	m_snakeLinkWidth = 0.5;
	m_snakeAttachDistance  = 0.01;
	
	
	m_goalX = 0.0;
	m_goalY = 0.0;
	m_goalRadius = 3.0;
	m_goalNrSides = 5;
	
	m_startState = NULL;
	
	m_tmax = 1.0;

	m_plannerGuideSeparationDistance = 10.0;
	m_plannerGuideWidth = 2.0;
	m_plannerGuideBorderExtendBefore = 4.0;
	m_plannerGuideBorderExtendAfter = 4.0;
	m_plannerGuideReachDistance = 1.5;

	m_plannerRegionsNrDimsX = 48;
	m_plannerRegionsNrDimsY = 48;

	m_plannerClearExponent = 6.0;
	m_plannerClearMax = 5.0;

	m_plannerUseRadix = false;
	m_plannerScaleRadix = 1000000000000;

	m_plannerMaxCountExploreAbandon = 50;
	m_plannerMaxCountFollowRetry = 400;
	m_plannerMaxCountFollowAbandon = 2000;

	m_plannerMinClearAcceptHint = 0.1;
	

	m_fromPointToStateSampleRadius = 2.0;	    
	m_fromPointToStateSampleNrAttempts = 50000;
	m_fromPointToStateSampleScaleAngle = 0.3;

	m_plannerRunAsMethod = 0;
	
}

~PlannerInput(void)
{
	if(m_occupancy)
	delete[] m_occupancy;
	if(m_startState)
	delete[] m_startState;
};


	int m_gridNrDimsX;
	int m_gridNrDimsY;
	double m_gridMinX;
	double m_gridMinY;
	double m_gridMaxX;
	double m_gridMaxY;
	int *m_occupancy;
	
	int m_snakeNrLinks;
	double m_snakeLinkLength;
	double m_snakeLinkWidth;
	double m_snakeAttachDistance;
	
	
	double m_goalX;
	double m_goalY;
	double m_goalRadius;
	int m_goalNrSides;
	
	double *m_startState;
	
	double m_tmax;

	/* Parameters to control the solution quality, i.e., how tight it follows
	 * discrete paths. The motion planner works by computing guides
	 * (grid paths to the goal) and then expanding a motion-tree
	 * along those paths.
	 *
	 * m_plannerGuideSeparationDistance:
	 *       Each discrete grid path is "regularized," by turning it
	 *       into a sequence of equidistant waypoints.  The separation
	 *      distance controls how far the waypoints should be from each
	 *     other. For the 80m x 80m scenes, a value of 10 seems to work well.
	 *     You may have to play with this value.
	 *
	 * m_plannerGuideWidth:
	 *     Defines the width of the corridor around the guide (grid path).
	 *     The smaller the value, the better the path, but may increase planning runtime.
	 *
	 * m_plannerGuideBorderExtendBefore:
	 * m_plannerGuideBorderExtendAfter:
	 *     It's best to assign the same values to these two variables. They define by how
	 *    much to extend the corridor before the previous waypoint and after the current waypoint.
	 *
	 * m_plannerGuideReachDistance:
	 *   Tolerance to determine when a waypoint is reached. Should always be <= m_plannerGuideWidth
	 *   Usually, somewhere between half and two thirds of m_plannerGuideWidth.
	 */
	double m_plannerGuideSeparationDistance; //if negative, discrete path is not transformed (this is ok too, and may work better in some scenarios)
	double  m_plannerGuideWidth;
	double m_plannerGuideBorderExtendBefore ;
	double m_plannerGuideBorderExtendAfter;
	double m_plannerGuideReachDistance;

	
	/* Parameters to tradeoff maximizing clearance/reducing distance
	 */
	double m_plannerClearExponent;
	double m_plannerClearMax;
	
	
	/* Parameters to control the runtime.
	 * The planner uses these parameters to determine how many regions to create.
	 * m_plannerRegionsNrDimsX should not exceed m_gridNrDimsX
	 * m_plannerRegionsNrDimsY should not exceed m_gridNrDimsY
	 *
	 * The parameters m_gridNrDimsX and m_gridNrDimsY determine the resolution
	 * for the occupancy grid.
	 *
	 * The idea is to set m_plannerRegionsNrDimsX and m_plannerRegionsNrDimsY
	 * to much smaller values. Try 64, and if that does not work increase to 80 or more.
	 */
	int m_plannerRegionsNrDimsX;
	int m_plannerRegionsNrDimsY;

	/* Speeding up the discrete search
	 * We have a radix implementation of Dijsktra's algorithm
	 * The radix implementation uses longs as oppsed to floats/double
	 * Scale should be set to a large value so not to loose precision
	 * Still hard to determine beforehand what is a good scale
	 * Set boolean variable to true if you would like to use radix
	 */
	bool m_plannerUseRadix;
	long m_plannerScaleRadix;

	/* Some other planner parameters just in case
	 * Default values should work ok
	 */
	int m_plannerMaxCountExploreAbandon;
	int m_plannerMaxCountFollowRetry;
	int m_plannerMaxCountFollowAbandon;
	

	/* Since the planner is called multiple times, it can be useful to let the planner
	 *  know what the previous solution was.
	 * If set, this vector should contain the remaining states from the current solution trajectory
	 * (the one the snake is following)
	 */
	std::vector<double> m_statesRemainingInCurrentSolution;

	/* These are used by RunMotionPlannerPointToGoal
	 * The objective there is to generate a valid state whose position
	 * is at or near a user-specified (x, y) position.
	 * So the user sets pin.m_startState[0] and pin.m_startState[1]
	 * The radius parameter controls the sampling range. If the user value does not work,
	 * the program will generate another start position within the circle with the given radius and
	 * pin.m_startState[0] and pin.m_startState[1] as the center
	 *
	 *NrAttempts: parameter to indicate how many to try sampling
	 *
	 *ScaleAngle: angles for the trailer links are generated from (-scale PI, scale PI).
	 * ScaleAngle value shouldpin.m_snakeNrLinks;    
			// m_sim.m_bodyLength = pin.m_snakeLinkLength;
			// m_sim.m_bodyWidth = pin.m_snakeLinkWidth;
			// m_sim.m_attachDist = pin.m_snakeAttachDistance;
			// m_sim.SetTimeStep(pin.m_dt_com);
			// m_sim.CompleteSetup();
			// // initialize the solution vectors
			// for (int i = 0; i < pin.m_robotNr; ++i) {
			// 	Solution *sol = new Solution();
			// 	sol->Clear();
			// 	m_sim.m_solutions.push_back(sol);
			// }
			// m_sim.m_maxTimeSte be > 0 and <= 1.0
	 *
	 *The sampler will try for a number of times (filling in all the state values) until
	 * a valid state is generated, or the maximum number of iterations is reached
	 */
	int m_fromPointToStateSampleRadius;
	int m_fromPointToStateSampleNrAttempts;
	double m_fromPointToStateSampleScaleAngle;

	/*Parameters for initializing planner with previous solution
	 */
	double m_plannerMinClearAcceptHint;
	
	//other params
	int m_plannerRunAsMethod;

	//just for graphics
	std::vector<double> m_decompRegions;	
};

struct RobotState {
	double *state;
};

class TrajOptPlanner {

    public:
		enum Status
	    {
			EXTEND_NORMAL    	= 0,
			EXTEND_NOPROGRESS 	= 1,
			EXTEND_GOAL      	= 2
	    };

		TrajOptPlanner(void){
			m_sim.SetScene(&m_scene);    
			m_decomp.SetScene(&m_scene);
			m_decomp.SetGoal(&m_goal);
			m_planner.SetDecomposition(&m_decomp);
			m_planner.SetSimulator(&m_sim);
		}
		
		
		virtual ~TrajOptPlanner(void)
		{
			// DeleteItems<GP::Solution *>(&m_solutionMR);
			// m_solutionMR.clear();
			// DeleteItems<GP::Solution *>(&m_sim.m_solutions);
			// m_sim.m_solutions.clear();
			m_decomp.Clear();
		}

		void SetupDecomposition(const PlannerInput &pin) {
			m_decomp.m_grid.Setup2D(pin.m_plannerRegionsNrDimsX, pin.m_plannerRegionsNrDimsY, pin.m_gridMinX, pin.m_gridMinY, pin.m_gridMaxX, pin.m_gridMaxY);
			m_decomp.m_dclearExp = pin.m_plannerClearExponent;
			m_decomp.m_dclearMax = pin.m_plannerClearMax;
			m_decomp.m_useRadix = pin.m_plannerUseRadix;
			m_decomp.m_radixScale = pin.m_plannerScaleRadix;
			
		}
		void SetupSimulator(const PlannerInput &pin) {
			m_sim.m_nrLinks = pin.m_snakeNrLinks;    
			m_sim.m_bodyLength = pin.m_snakeLinkLength;
			m_sim.m_bodyWidth = pin.m_snakeLinkWidth;
			m_sim.CompleteSetup();	    
		}
		
		void SetupScene(const PlannerInput &pin) {
			m_scene.m_grid.Setup2D(pin.m_gridNrDimsX, pin.m_gridNrDimsY, pin.m_gridMinX, pin.m_gridMinY, 
										pin.m_gridMaxX, pin.m_gridMaxY);
		}

		void SetupGoal(const PlannerInput &pin) {
			m_goal.SetAsCircle(pin.m_goalX, pin.m_goalY, pin.m_goalRadius, pin.m_goalNrSides);
		}

		void SetupStartState(const PlannerInput &pin) {
			m_sim.SetState(pin.m_startState);	    
		}

		void SetupOccupancyGrid(const PlannerInput &pin) {
			for(int i = m_scene.m_grid.GetNrCells() - 1; i >= 0; --i)
			if(pin.m_occupancy[i] == GRID_CELL_OBSTACLE)
				m_scene.AddCellObstacle(i);
			else if(pin.m_occupancy[i] == GRID_CELL_FREE)
				m_scene.AddCellFree(i);
			else
				m_scene.AddCellUnknown(i);
		}
		
		void SetupPlanner(const PlannerInput &pin)
		{
			m_planner.m_guideWidth = pin.m_plannerGuideWidth;
			m_planner.m_guideSeparationDistance = pin.m_plannerGuideSeparationDistance;
			m_planner.m_guideBorderExtendBefore = pin.m_plannerGuideBorderExtendBefore;
			m_planner.m_guideBorderExtendAfter = pin.m_plannerGuideBorderExtendAfter;
			m_planner.m_follow.SetReachTolerance(pin.m_plannerGuideReachDistance);

			m_planner.m_maxCountExploreAbandon = pin.m_plannerMaxCountExploreAbandon;
			m_planner.m_maxCountFollowRetry = pin.m_plannerMaxCountFollowRetry;
			m_planner.m_maxCountFollowAbandon = pin.m_plannerMaxCountFollowAbandon;

			m_planner.m_minClearAcceptHint = pin.m_plannerMinClearAcceptHint;
			
			m_planner.m_runAsMethod = pin.m_plannerRunAsMethod;
			
		}
		
		
		void Setup(const PlannerInput &pin)
		{
			SetupScene(pin);
			SetupSimulator(pin);
			SetupDecomposition(pin);	    
			SetupGoal(pin);
			SetupStartState(pin);
			SetupOccupancyGrid(pin);
			SetupPlanner(pin);	    
		}

		bool IsStartConnectedToGoalViaGridPath(void)
		{
			auto rid = m_decomp.LocateRegion(m_sim.GetState());
			return rid >= 0 && m_decomp.GetRegionById(rid)->m_pathDataToGoal.m_hasPath;
		}
		
		void InitializeAndRunPlanner(const PlannerInput &pin, Solution &sol)
		{
			m_planner.InitializeWithSolution(pin.m_statesRemainingInCurrentSolution);
			m_planner.Run(pin.m_tmax);
			m_planner.GetSolution(&sol);
		}

		// planning functions
		virtual void Plan(const PlannerInput &pin, Solution &sol);

		
		virtual void SnakePlacement(const PlannerInput &pin, const double s[], double pts[]);

			
		virtual void RunMotionPlannerStartToGoal(PlannerInput &pin,  Solution &sol);
		// virtual void RunMotionPlannerStartToGoal(Solution &sol);
		/* RunMotionPlannerPointToGoal is similar to RunMotionPlannerStartToGoal
		* However, in RunMotionPlannerPointToGoal, the user does not need to set all the values for pin.m_startState --
		* the user must allocate memory for pin.m_startState and just set its x and y values (positions 0 and 1), i.e.,
		* set value for pin.m_startState[0] and pin.m_startState[1]
		* The function will then generate random values for the remaining state values (repeat the process until not in collision),
		* and then plan collision-free and dynamically-feasible motion trajectory from pin.m_startState to the goal
		*/
		// virtual void RunMotionPlannerPointToGoal(PlannerInput &pin,  Solution &sol);

		

		private:
		MPSnakeSimulator 	m_sim;   
		Goal 				m_goal;    
		SceneCells 			m_scene;
		GridDecomposition 	m_decomp;
		FollowPlanner 		m_planner;
		Archives 			m_archives; 
		Params 				m_params;
		Sensor 				m_sensor;	
		int 				m_stateDim;
		double 				m_goalRadius;
		double 				m_goalNrSides;
		double 				*m_goals;
		double 				*m_initState;
		double 				m_tmax_aRobot;
		double 				m_dt_com;
		// bool 				m_permuation;
	};


}
#endif
