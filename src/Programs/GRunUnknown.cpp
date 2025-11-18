#include "Utils/GManager.hpp"
#include "GP/Setup.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Stats.hpp"
#include "Utils/GTexture.hpp"
#include "GP/Sensor.hpp"
#include "TrajOpt/DevPlanner.hpp"
#include "GP/MPSnakeSimulator.hpp"
#include "GP/Setup.hpp"

namespace GP
{	
    class GRunUnknownManager : public GManager
    {
    public:
	enum
	    {
		FLAG_DRAW_SCENE          = 1,
		FLAG_DRAW_SOLUTION = 2,
		FLAG_DRAW_CURR_STATE = 4,
		FLAG_DRAW_SENSOR = 8,
		FLAG_DRAW_GOAL = 16,
		FLAG_DRAW_SOLUTION_FARTHEST = 32,
		FLAG_DRAW_DECOMP = 64,
		FLAG_RUN = 128
	    };
	
	GRunUnknownManager(void) : GManager()
	{
	    m_flags  = AddFlag(0, FLAG_DRAW_SCENE | FLAG_DRAW_SOLUTION | FLAG_DRAW_CURR_STATE | FLAG_DRAW_SENSOR | FLAG_DRAW_SOLUTION_FARTHEST|FLAG_DRAW_GOAL|FLAG_DRAW_DECOMP);
	    
	    m_sensor.m_scene = &m_sceneKnownObstacles;
	    
	    m_trajPos = 0;
	    m_trajDist = 0.0;
	    
	    m_pClicked[0] = m_pClicked[1] = 0.0;

	    m_currState.resize(100);
	    m_goal.resize(4);

	    std::fill(m_currState.begin(), m_currState.end(), 0);
	    m_currState[0] = -33.826965;
	    m_currState[1] = -33.491208;

	    m_goal[0] = 21.0;
	    m_goal[1]= 32.0;
	    m_goal[2] = 3.0;
	    m_goal[3] = 5;

	    m_remainingPos = -1;
	    
	}
	
	virtual ~GRunUnknownManager(void)
	{
	}

	
	virtual void HandleEventOnDisplay(void)
	{
	    GManager::HandleEventOnDisplay();
	    GDraw2D();
	    
	    const double *pmin = m_sceneKnownObstacles.GetGrid()->GetMin();
	    const double *pmax = m_sceneKnownObstacles.GetGrid()->GetMax();
	    const double  off  = 0.001;
	    double        rgb[3];
	    double bboxCell[4];
	    GMaterial     gmat;
	    TriMesh tmesh;
	    Polygon2D poly;
	    
	    
	    m_gtex.AutomaticCoords();
	    m_gtex.Use();
	    gmat.SetTurquoise();
	    gmat.SetDiffuse(0.0,0.0, 0.0);
		 
	    
	    SetValue(INDEX_MINX, pmin[0] - off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MINY, pmin[1] - off * (pmax[1] - pmin[1]));
	    SetValue(INDEX_MAXX, pmax[0] + off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MAXY, pmax[1] + off * (pmax[1] - pmin[1]));


	    if(HasFlag(m_flags, FLAG_DRAW_SCENE))
	    {
		if(HasFlag(GManager::m_flags, FLAG_3D) == false)
		{
		    GDrawColor(0.7, 0.7, 0.7);
		    //  glEnable(GL_TEXTURE_2D);
		    
		    for(auto & cid : m_sceneKnownObstacles.m_cidsObstacles)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawAABox2D(bboxCell[0], bboxCell[1],  bboxCell[2], bboxCell[3]);		
		    }
		    GDrawColor(1.0, 1.0, 1.0);
		    for(auto & cid : m_sceneKnownObstacles.m_cidsUnknown)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawAABox2D(bboxCell[0], bboxCell[1],  bboxCell[2], bboxCell[3]);		
		    }
		    GDrawColor(0.9, 0.9, 0.9);
		    for(auto & cid : m_sceneKnownObstacles.m_cidsFree)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawAABox2D(bboxCell[0], bboxCell[1],  bboxCell[2], bboxCell[3]);		
		    }
		}
		else
		{
		    GDraw3D();
		    gmat.SetAmbient(1.0, 0.7, 0.7);
		    GDrawMaterial(&gmat);
		    m_sceneKnownObstacles.DrawObstacles();
		    GDraw2D();
		    GDrawColor(0.8, 0.8, 0.8);
		    GDrawAABox2D(pmin, pmax);		    
		}		
	    }

	    if(HasFlag(GManager::m_flags, FLAG_3D) == false)
	    {
		GDrawLineWidth(4.0);
		GDrawWireframe(true);
		GDrawColor(0.3, 0.3, 0.3);
		GDrawAABox2D(m_sceneKnownObstacles.GetGrid()->GetMin(), m_sceneKnownObstacles.GetGrid()->GetMax());
		GDrawWireframe(false);
		GDrawLineWidth(1.0);
	    }
	    
	    if(HasFlag(m_flags, FLAG_DRAW_SENSOR))
	    {
		if(HasFlag(GManager::m_flags, FLAG_3D) == false)
		{
		    auto cids = m_sensor.GetSensedOccupiedCells();
		    GDrawColor(0.1, 0.1, 0.1);		
		    for(auto & cid : *cids)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawAABox2D(bboxCell[0], bboxCell[1], bboxCell[2], bboxCell[3]);		
		    }
		    
		    cids = m_sensor.GetSensedFreeCells();
		    GDrawColor(0, 1.0, 0.0);		
		    for(auto & cid : *cids)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawAABox2D(bboxCell[0], bboxCell[1],  bboxCell[2], bboxCell[3]);		
		    }
		}
		else
		{
		    GDraw3D();
		    
		    auto cids = m_sensor.GetSensedOccupiedCells();
		    gmat.SetAmbient(0.1, 0.1, 0.1);
		    GDrawMaterial(&gmat);
		    for(auto & cid : *cids)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawBox3D(bboxCell[0], bboxCell[1], 0.0, bboxCell[2], bboxCell[3], 1.1);		
		    }
		    
		    cids = m_sensor.GetSensedFreeCells();
		    gmat.SetAmbient(0.0, 1.0, 0.8);
		    GDrawMaterial(&gmat);		    
		    for(auto & cid : *cids)
		    {
			m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
			GDrawBox3D(bboxCell[0], bboxCell[1],  0.0, bboxCell[2], bboxCell[3], 0.01);		
		    }

		    GDraw2D();
		}
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_GOAL))
	    {
		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.3);
		GDrawColor(0, 0, 1);
		GDrawRegularPolygon2D(m_goal[0], m_goal[1], m_goal[2], m_goal[3]);
		GDrawPopTransformation();
	    }
	    
	    
	    if(HasFlag(m_flags, FLAG_DRAW_SOLUTION))
	    {
		const double rgb1[] = {1.0, 0.0, 1.0};
		const double rgb2[] = {0.0, 1.0, 1.0};

		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.03);
		DrawSolution(m_solution, rgb1, rgb2, m_remainingPos);
		GDrawPopTransformation();
	    }

	    
	    if(HasFlag(m_flags, FLAG_DRAW_SOLUTION_FARTHEST))
	    {
		const double rgb1[] = {1.0, 0.0, 0.0};
		const double rgb2[] = {0.9, 0.7, 0.7};
		const double rgb3[] = {0.0, 0.0, 1.0};
		const double rgb4[] = {0.7, 0.7, 0.9};

		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.03);
		DrawSolution(m_solutionCurrToFarthest, rgb1, rgb2);
		DrawSolution(m_solutionFarthestToCurr, rgb3, rgb4);
		GDrawPopTransformation();
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_DECOMP))
	    {
		GDraw2D();
		GDrawWireframe(true);
		GDrawLineWidth(2.0);		
		GDrawColor(1.0, 1.0, 0.0);
		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.1);
		for(int i = 0; i < m_pin.m_decompRegions.size(); i += 4)
		    GDrawAABox2D(&(m_pin.m_decompRegions[i]));
		GDrawPopTransformation();		
		GDrawWireframe(false);		
	    }
	    
	    
	    if(HasFlag(m_flags, FLAG_DRAW_CURR_STATE))
	    {
		if(HasFlag(GManager::m_flags, FLAG_3D) == false)
		{
		    GDrawPushTransformation();
		    GDrawMultTrans(0, 0, 0.3);
		    GDrawColor(0, 1, 0);
		    for(int i = 0; i < m_currSnakePlacement.size(); i += 8)
		    {
			if(i == 0)
			    GDrawColor(1, 0, 0);
			else
			    GDrawColor(0, 0, 1);
			GDrawQuad2D(&m_currSnakePlacement[i]);
		    }		
		    GDrawPopTransformation();
		}
		else
		{
		    GDraw3D();
		    
		    for(int i = 0; i < m_currSnakePlacement.size(); i += 8)
		    {
			if(i == 0)
			    gmat.SetAmbient(1, 0, 0);
			else
			    gmat.SetAmbient(0.0, 0.0, 0.7);
			poly.Clear();
			poly.AddVertices(4, &m_currSnakePlacement[i]);
			tmesh.Clear();
			tmesh.AddExtrudedPolygon(&poly, 0.0, 0.75);
			GDrawMaterial(&gmat);
			tmesh.Draw();
		    }	
		    GDraw2D();
		    
		}
		
	    }
/*
	    char msg[300];
	    const int n = 10;

	    GDrawColor(1, 0, 0);
	    auto xunit = (int) ((pmax[0] - pmin[0]) / n);
	    auto yunit = (int) ((pmax[1] - pmin[1]) / n);
	    
	    for(int i = 1; i < n; ++i)
	    {
		auto x = pmin[0] + i * xunit;		
		sprintf(msg, "%4.2f", x);
		GDrawString2D(msg, x, pmin[1] + 0.05 * (pmax[1] - pmin[1]));
	    }
	    for(int i = 1; i < n; ++i)
	    {
		auto y = pmin[1] + i * yunit;		
		sprintf(msg, "%4.2f", y);
		GDrawString2D(msg, pmin[0] + 0.05 * (pmax[0] - pmin[0]), y);
	    }
*/
	    char msg1[300];
	    char msg2[300];
	    GDrawColor(1, 0, 0);	    

	    sprintf(msg1, "(%d,%d)", (int) pmin[0], (int) pmin[1]);
	    sprintf(msg2, "(%d,%d)", (int) pmax[0], (int) pmax[1]);

	    if(HasFlag(GManager::m_flags, FLAG_3D) == false)
	    {
		GDrawString2D(msg1, pmin[0] + 5.5, pmin[1] + 10);
		GDrawString2D(msg2, pmax[0] - 50, pmax[1] - 15); //100, 20
	    }
	    else
	    {
		GDrawString3D(msg1, pmin[0] + 5.5, pmin[1] + 1.25, 0.0, false);
		GDrawString3D(msg2, pmax[0] - 4.5, pmax[1] - 2, 0.0, false);
	    }
	    
	}

	virtual void DrawSolution(const Solution &sol, const double rgb1[], const double rgb2[], const int remPos = 0)
	{
	    	auto dimState = sol.m_dimState;
		
		GDrawLineWidth(8.0);

		GDrawColor(rgb1[0], rgb1[1], rgb1[2]);
		for(int i = std::max(1, remPos); i < sol.TrajGetNrStates(); ++i)
		    GDrawSegment2D(sol.TrajGetState(i-1), sol.TrajGetState(i));
		
		GDrawColor(0, 1, 1);
		for(int i = 1; i < sol.RemainingPathGetNrPoints(); ++i)
		    GDrawSegment2D(sol.RemainingPathGetPoint(i-1), sol.RemainingPathGetPoint(i));
		

		GDrawLineWidth(1.0);
		
	}
	
	
	virtual void HandleEventOnTimer(void)
	{
	    if(!HasFlag(m_flags, FLAG_RUN) )
		return;
	    RunOnce();
	    
	}

	virtual bool HandleEventOnNormalKeyPress(const int key)
	{
		return true;
	//     if(key == 'p')
	// 	m_flags = FlipFlag(m_flags, FLAG_RUN);
	//     else if(key == 'd')
	// 	m_flags = FlipFlag(m_flags, FLAG_DRAW_DECOMP);
	//     else if(key == 'r')
	// 	RunOnce();	    
	//     else if(key == 'e')
	// 	RunPlannerToExplore();
	//     else if(key == 'f')
	// 	RunPlannerToFrontier();
	//     else if(key == 's')
	//     {
	// 	printf("SnakeStartX %f\n",  m_currState[0]);
	// 	printf("SnakeStartY %f\n", m_currState[1]);
	// 	for(int i = 0; i < m_pin.m_snakeNrLinks; ++i)
	// 	    printf("SnakeStartTrailer%dTheta %f\n", i,  m_currState[5 + i]);		
	//     }
	//     else if(key == '+')
	// 	MoveAlongSolution();
	    
	    
	    	    
	//     return GManager::HandleEventOnNormalKeyPress(key);
	}

	

	
	virtual void RunOnce(void)
	{
		return;
	//     if(Algebra2D::PointDist(&m_currState[0], &m_goal[0]) <= m_goal[2])
	// 	return;
	    
	//     auto hasNew = Sense();

	 
	//     if(hasNew ||  (m_solution.TrajGetNrStates() == 0 || /*m_trajDist > 0.25 ||*/ m_trajPos >= m_solution.TrajGetNrStates()))
	// 	RunPlannerToGoal();
	//     else if(hasNew == false && (m_solution.TrajGetNrStates() == 0 || m_trajPos >= m_solution.TrajGetNrStates()))
	// 	RunPlannerToGoal();
	    
	//     MoveAlongSolution();
	}
	

	virtual void RunPlannerToGoal(void)
	{
		return;
	//     bool useSensor = true;
	    
	//     PrepareOccupancy(m_pin, useSensor);
	    
	//     //start state
	//     auto dimState = m_pin.m_snakeNrLinks + 5;
	//     //pin.m_startState = new double[dimState];
	//     for(int i = 0; i < dimState; ++i)
	// 	m_pin.m_startState[i] = m_currState[i];
	    
	//     //goal
	//     m_pin.m_goalX = m_goal[0];
	//     m_pin.m_goalY = m_goal[1];
	//     m_pin.m_goalRadius = m_goal[2];
	//     m_pin.m_goalNrSides = (int) m_goal[3];
	    
	    
	//     //solution if it exists
	//     if(m_remainingPos >= 0 && m_remainingPos < m_solution.TrajGetNrStates())
	// 	m_pin.m_statesRemainingInCurrentSolution.assign(m_solution.m_trajStates.begin() + m_remainingPos * m_solution.m_dimState, m_solution.m_trajStates.end());
	//     else
	// 	m_pin.m_statesRemainingInCurrentSolution.clear();
	    
	    
	//     RunMotionPlannerStartToGoal(m_pin, m_solution);
	//     m_trajPos = 1;
	//     m_trajDist = 0.0;
	//     m_remainingPos = -1;
	    
	//     GP::Stats::GetSingleton()->PrintValues(stdout);
	}

	
	virtual void RunPlannerToFrontier(void)
	{
	    return;
		// bool useSensor = false;
	    
	    // PrepareOccupancy(m_pin, useSensor);
	    
	    // //start state
	    // auto dimState = m_pin.m_snakeNrLinks + 5;
	    // //pin.m_startState = new double[dimState];
	    // for(int i = 0; i < dimState; ++i)
		// m_pin.m_startState[i] = m_currState[i];
	    
	    // //goal
	    // m_pin.m_goalX = m_goal[0];
	    // m_pin.m_goalY = m_goal[1];
	    // m_pin.m_goalRadius = m_goal[2];
	    // m_pin.m_goalNrSides = (int) m_goal[3];
	    
	    
	    // m_pin.m_statesRemainingInCurrentSolution.clear();
	    // RunMotionPlannerPointToGoal(m_pin, m_solution);
	    // m_trajPos = 1;
	    // m_trajDist = 0.0;
	    // m_remainingPos = -1;
	    // GP::Stats::GetSingleton()->PrintValues(stdout);
	}
	
	
	virtual bool Sense(void)
	{
	    auto data = m_sensor.Sense(&m_currState[0]);
	    bool hasNew = false;
	    
	    if(data->m_cellsFree != m_sensedLast.m_cellsFree ||
	       data->m_cellsOccupied != m_sensedLast.m_cellsOccupied)
	    {
		hasNew = true;
		m_sensedLast = *data;
	    }
	    
	    //return hasNew;
	    
	    return data->m_hasNew;
	    
	}
	
	virtual bool MoveAlongSolution(void)
	{
	    return true;
		// auto dimState = m_solution.m_dimState;
	    // if(m_trajPos < 0 || m_trajPos >= m_solution.TrajGetNrStates())
		// return false;
	    // auto s = m_solution.TrajGetState(m_trajPos);
	    // m_trajDist += Algebra2D::PointDist(&m_currState[0], s);
	    // for(int i = 0; i < dimState; ++i)
		// m_currState[i] = s[i];
	    // m_remainingPos = m_trajPos;
	    
	    // SnakePlacement(m_pin, &m_currState[0], &m_currSnakePlacement[0]);
  
	    // ++m_trajPos;
	    
	    // return true;
	    
	}
	
	
	virtual void PrepareOccupancy(PlannerInput &pin, const bool useSensor = true)
	{
	    pin.m_gridNrDimsX = m_sceneKnownObstacles.GetGrid()->GetDims()[0];
	    pin.m_gridNrDimsY = m_sceneKnownObstacles.GetGrid()->GetDims()[1];
	    pin.m_gridMinX = m_sceneKnownObstacles.GetGrid()->GetMin()[0];
	    pin.m_gridMinY = m_sceneKnownObstacles.GetGrid()->GetMin()[1];
	    pin.m_gridMaxX = m_sceneKnownObstacles.GetGrid()->GetMax()[0];
	    pin.m_gridMaxY = m_sceneKnownObstacles.GetGrid()->GetMax()[1];
	    
	    //pin.m_occupancy = new int[pin.m_gridNrDimsX * pin.m_gridNrDimsY];
	    std::fill(pin.m_occupancy, pin.m_occupancy + pin.m_gridNrDimsX * pin.m_gridNrDimsY, GP::GRID_CELL_UNKNOWN);

	    if(useSensor)
	    {
		auto cids = m_sensor.GetSensedOccupiedCells();
		for(auto & cid : *cids)
		    pin.m_occupancy[cid] = GRID_CELL_OBSTACLE;
		cids = m_sensor.GetSensedFreeCells();
		for(auto & cid : *cids)
		    pin.m_occupancy[cid] = GRID_CELL_FREE;
	    }
	    else
	    {
		auto cids = &(m_sceneKnownObstacles.m_cidsObstacles);
		for(auto & cid : *cids)
		    pin.m_occupancy[cid] = GRID_CELL_OBSTACLE;
		cids = &(m_sceneKnownObstacles.m_cidsFree);
		for(auto & cid : *cids)
		    pin.m_occupancy[cid] = GRID_CELL_FREE;	  
	    }
	    
	}
	
	PlannerInput m_pin;	
	ScenePolys m_sceneKnownObstacles;	
	Sensor m_sensor;
	Sensor::Data m_sensedLast;
	
	Solution m_solution;
	Solution m_solutionCurrToFarthest;
	Solution m_solutionFarthestToCurr;
	std::vector<double> m_currState;
	std::vector<double> m_currSnakePlacement;	
	std::vector<double> m_goal;
	double m_pClicked[2];

	int m_remainingPos;
	
	
	int m_trajPos;
	double m_trajDist;
	
	Flags            m_flags;
	GTexture         m_gtex;
	GTexture  m_gtexTerrain;
    };
};


// extern "C" int GRunUnknown(int argc, char **argv)
// {
//     GP::GRunUnknownManager gManager;
//     GP::Params params;
    
//     printf("usage: ./bin/Runner GRunUnknown fnameParams\n");
    
//     if(argc < 1)
// 	return 0;
       
//     FILE *in = fopen(argv[1], "r");
//     params.Read(in);
//     fclose(in);
//     params.ParseArgs(2, argc-1, argv);
//     params.Print(stdout);


   
//     gManager.SetupFromParams(&params);
//     GP::GDrawSetupFromParams(&params);

    
//     GP::Setup::SetupPlannerInput(params, gManager.m_pin);
//     printf("The size of the decomposition %d \n", gManager.m_pin.m_decompRegions.size());


//     //setup sensor
//     gManager.m_sensor.SetRadius(params.GetValueAsDouble("SensorRadius", 300));
  
//     //setup scene with known obstacles
//     gManager.m_sceneKnownObstacles.SetupFromParams(&params);

//      auto fname = params.GetValue("SceneOccupancyFile", NULL);
//      if(fname)
// 	 gManager.m_sceneKnownObstacles.ReadOccupancyGrid(fname);
//      else if((fname = params.GetValue("SceneObstaclesFile", NULL)))
// 	     gManager.m_sceneKnownObstacles.ReadObstacles(fname);
//     gManager.m_sceneKnownObstacles.AddBoundary(params.GetValueAsDouble("SceneBoundaryThickness", GP::Constants::SCENE_BOUNDARY_THICKNESS));

//     auto dimState = gManager.m_pin.m_snakeNrLinks + 5;
//     for(int i = 0; i < dimState; ++i)
// 	gManager.m_currState[i] = gManager.m_pin.m_startState[i];
//     gManager.m_goal[0] = gManager.m_pin.m_goalX;
//     gManager.m_goal[1] = gManager.m_pin.m_goalY;
//     gManager.m_goal[2] = gManager.m_pin.m_goalRadius;
//     gManager.m_goal[3] = gManager.m_pin.m_goalNrSides;
//     printf("The size of the decomposition %d \n", gManager.m_pin.m_decompRegions.size());
    
//     gManager.m_currSnakePlacement.resize(8 * (gManager.m_pin.m_snakeNrLinks + 1));    
//     SnakePlacement(gManager.m_pin, &gManager.m_currState[0], &gManager.m_currSnakePlacement[0]);
    
		   
//     gManager.m_gtex.SetFileName("textures/terrain2.ppm");
//     gManager.m_gtexTerrain.SetFileName("textures/terrain2.ppm");
    
//     gManager.Help();
//     gManager.MainLoop("GRunDecomp", 
// 		      params.GetValueAsInt("GraphicsResolutionX", GP::Constants::GRAPHICS_RESOLUTION_X),
// 		      params.GetValueAsInt("GraphicsResolutionY", GP::Constants::GRAPHICS_RESOLUTION_Y));
    
//     return 0;
// }
