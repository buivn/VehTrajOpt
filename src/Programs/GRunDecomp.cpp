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
#include "GP/GridDecomposition.hpp"
// #include "GP/InterfacePlanner.hpp"

namespace GP
{	
    class GRunDecompManager : public GManager
    {
    public:
	enum
	    {
		FLAG_DRAW_SCENE          = 1,
		FLAG_DRAW_DECOMP_REGIONS = 2,
		FLAG_DRAW_DECOMP_EDGES   = 4,
		FLAG_DRAW_CELLS_OBSTACLES = 8,
		FLAG_DRAW_CLICKED = 16,
		FLAG_DRAW_GOAL = 32
	    };
	
	GRunDecompManager(void) : GManager()
	{
	    m_flags  = AddFlag(0, FLAG_DRAW_SCENE | FLAG_DRAW_DECOMP_REGIONS  | FLAG_DRAW_GOAL);

	    m_decomp.SetScene(&m_sceneCells);
	    m_decomp.SetGoal(&m_goal);
	    
	    m_rid = m_ridFarthest = -1;
	    m_pClicked[0] = m_pClicked[1] = 0.0;
	    m_dclearMax = 0.0;

	    m_goal.SetAsCircle(34, 34, 3.0, 5);
	    
	}
	
	virtual ~GRunDecompManager(void)
	{
	}

	
	virtual void HandleEventOnDisplay(void)
	{
	    GManager::HandleEventOnDisplay();
	    const bool    is2D = GDrawIs2D();
	    const double *pmin = m_sceneKnownObstacles.GetGrid()->GetMin();
	    const double *pmax = m_sceneKnownObstacles.GetGrid()->GetMax();
	    const double  off  = 0.002;
	    double        rgb[3];
	    double bboxCell[4];
	    
	    GMaterial     gmat;

	    GDraw2D();
	    
	    SetValue(INDEX_MINX, pmin[0] - off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MINY, pmin[1] - off * (pmax[1] - pmin[1]));
	    SetValue(INDEX_MAXX, pmax[0] + off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MAXY, pmax[1] + off * (pmax[1] - pmin[1]));


	    if(HasFlag(m_flags, FLAG_DRAW_SCENE))
	    {
		GDrawColor(0.7, 0.7, 0.7);
		for(auto & cid : m_sceneKnownObstacles.m_cidsObstacles)
		{
		    m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
		    GDrawAABox2D(bboxCell[0], bboxCell[1],  bboxCell[2], bboxCell[3]);		
		}
	    }
	    
	    GDrawLineWidth(4.0);
	    GDrawWireframe(true);
	    GDrawColor(0.3, 0.3, 0.3);
	    GDrawAABox2D(m_sceneKnownObstacles.GetGrid()->GetMin(), m_sceneKnownObstacles.GetGrid()->GetMax());
	    GDrawWireframe(false);
	    GDrawLineWidth(1.0);
	    
	    
	    if(HasFlag(m_flags, FLAG_DRAW_DECOMP_REGIONS))
	    {
		GDrawWireframe(true);
		GDraw2D();
		
		//GDrawColor(0, 0, 0);
		GDrawLineWidth(4.0);
		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.15);
		
		for(int i = m_decomp.GetNrRegions() - 1; i >= 0; --i)
		{
		    auto r = dynamic_cast<GridRegion*>(m_decomp.GetRegionById(i));

		    if(r->m_children.empty() == false)
			continue;
		    
		    if(r->m_cidsObstacles.empty())
			GDrawColor(0, 1, 0);
		    else
			GDrawColor(1, 0, 0);
		    auto offx = 0.05 * (r->m_bbox[2] - r->m_bbox[0]);
		    auto offy = 0.05 * (r->m_bbox[3] - r->m_bbox[1]);
		    
		    //	    GDrawAABox2D(r->m_bbox[0] + offx,  r->m_bbox[1] + offy, r->m_bbox[2] - offx, r->m_bbox[3] - offy);

		    
		    if(r->m_cidsObstacles.empty())
		    {

			//auto c = r->m_dclear / m_dclearMax;
			//GDrawColor(c, c, 0.0);
			GDrawAABox2D(r->m_bbox);

			GDrawColor(0, 0, 0);
			GDrawWireframe(true);
			GDrawAABox2D(r->m_bbox);
		  
		    }
		    
		    GDrawWireframe(false);
		    
		}
		GDrawPopTransformation();
		GDrawWireframe(false);
		GDrawLineWidth(1.0);
		
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_DECOMP_EDGES))
	    {
		GDrawWireframe(true);
		GDraw2D();
		
		GDrawColor(0, 0, 1.0);
		GDrawLineWidth(4.0);
		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.25);
		m_decomp.DrawEdges();
		GDrawPopTransformation();
		GDraw3D();
		GDrawWireframe(false);
		GDrawLineWidth(1.0);
		
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_CLICKED))
	    {
		gmat.SetPearl();
		gmat.SetDiffuse(1.0, 0.0, 0);
		GDrawMaterial(&gmat);
		//	GDrawSphere3D(m_pClicked[0], m_pClicked[1], 0.25, 0.5);

		if(m_rid >= 0)
		{
		    auto r = dynamic_cast<GridRegion*>(m_decomp.GetRegionById(m_rid));
		    GDrawBox3D(r->m_bbox[0], r->m_bbox[1], 0.0, r->m_bbox[2], r->m_bbox[3], 2.25);
		}
		
		gmat.SetDiffuse(1.0, 1.0, 0);
		GDrawMaterial(&gmat);
		if(m_ridFarthest >= 0)
		{
		    auto r = dynamic_cast<GridRegion*>(m_decomp.GetRegionById(m_ridFarthest));
		    GDrawBox3D(r->m_bbox[0], r->m_bbox[1], 0.0, r->m_bbox[2], r->m_bbox[3], 6.25);
		}
		
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_GOAL))
	    {
		GDraw2D();
		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.3);

		//draw goal
		GDrawColor(0, 0, 1);
		m_goal.Draw();

		//draw goal connections
		/*auto gconns = &(m_decomp.m_goalConnections);
		auto neighs = gconns->GetNeighbors();
		GDrawColor(0, 1, 0);
		GDrawLineWidth(4.0);
		for(int i = gconns->GetNrNeighbors() - 1; i >= 0; --i)
		{
		    auto rneigh = dynamic_cast<GridRegion*>(m_decomp.GetRegionById(neighs[i]));
		    GDrawSegment2D(rneigh->GetCentroid(), m_goal.GetPolygon()->GetCentroid());
		    
		    }*/
	
		//draw path to goal
		if(m_rid >= 0)
		{
		    GDrawColor(1, 0, 0);
		    
		    auto r = dynamic_cast<GridRegion*>(m_decomp.GetRegionById(m_rid));
		    auto path = &(r->m_pathDataToGoal.m_pts);
		    for(int i = 2; i < path->size(); i += 2)
			GDrawSegment2D(&(*path)[i-2], &(*path)[i]);
		    
		}

			GDrawLineWidth(1.0);

		GDrawPopTransformation();
		
	    }
	    
	    
	}	

	virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
	{
	    double pos[3];
	    
	    MousePosFromScreenToWorld(x, y, &pos[0], &pos[1], &pos[2]);
	    printf("clicked at %f %f %f\n", pos[0], pos[1], pos[2]);
	    m_rid = m_decomp.LocateRegion(pos);
	    if(m_rid >= 0)
		m_ridFarthest = m_decomp.FarthestRegionInUnknownSpace(m_rid);

	    printf("rids = %d %d\n", m_rid, m_ridFarthest);
	    
	    printf("rid = %d %p %p ridFarthest = %d %p %p\n",
		   m_rid, 
		   m_rid >= 0 ? m_decomp.GetRegionById(m_rid) : NULL,
		   m_rid >= 0 ? dynamic_cast<GridRegion*>(m_decomp.GetRegionById(m_rid)) : NULL,
		   m_ridFarthest,
		   m_ridFarthest >= 0 ? m_decomp.GetRegionById(m_ridFarthest) : NULL,
		   m_ridFarthest >= 0 ? dynamic_cast<GridRegion*>(m_decomp.GetRegionById(m_ridFarthest)) : NULL);
	    
		   
	    m_pClicked[0] = pos[0];
	    m_pClicked[1] = pos[1];

	    return true;
	}
	
	virtual bool HandleEventOnNormalKeyPress(const int key)
	{
	    if(key == 's')
		m_flags = FlipFlag(m_flags, FLAG_DRAW_SCENE);
	    else if(key == 'r')
		m_flags = FlipFlag(m_flags, FLAG_DRAW_DECOMP_REGIONS);
	    else if(key == 'e')
		m_flags = FlipFlag(m_flags, FLAG_DRAW_DECOMP_EDGES);
	     else if(key == 'o')
		m_flags = FlipFlag(m_flags, FLAG_DRAW_CELLS_OBSTACLES);
	    else if(key == 'c')
		m_flags = FlipFlag(m_flags, FLAG_DRAW_CLICKED);
	    
	    return GManager::HandleEventOnNormalKeyPress(key);
	}

	SceneCells m_sceneCells;	
	ScenePolys m_sceneKnownObstacles;
	GridDecomposition m_decomp;
	Goal m_goal;
	
	double m_pClicked[2];
	int m_rid;
	int m_ridFarthest;
	
	double m_dclearMax;
	
	
	Flags            m_flags;
	GTexture         m_gtex;
	GTexture  m_gtexTerrain;
    };
};


extern "C" int GRunDecomp(int argc, char **argv)
{
    GP::GRunDecompManager gManager;
    GP::Params params;
    GP::PlannerInput pin;
    
    printf("usage: ./bin/Runner GRunDecomp fnameParams\n");
    
    if(argc < 1)
	return 0;
       
    FILE *in = fopen(argv[1], "r");
    params.Read(in);
    fclose(in);
    params.ParseArgs(2, argc-1, argv);
    params.Print(stdout);

    
    gManager.SetupFromParams(&params);
    GP::GDrawSetupFromParams(&params);

    GP::Setup::SetupPlannerInput(params, pin);

    gManager.m_goal.SetAsCircle(pin.m_goalX, pin.m_goalY, pin.m_goalRadius, pin.m_goalNrSides);
    printf("GOAL = %f %f : %f %f\n", pin.m_goalX, pin.m_goalY, gManager.m_goal.GetPolygon()->GetCentroid()[0], gManager.m_goal.GetPolygon()->GetCentroid()[1]);
    
    

    //setup scene with known obstacles
    gManager.m_sceneKnownObstacles.SetupFromParams(&params);
     auto fname = params.GetValue("SceneOccupancyFile", NULL);
     if(fname)
	 gManager.m_sceneKnownObstacles.ReadOccupancyGrid(fname);
     else if((fname = params.GetValue("SceneObstaclesFile", NULL)))
	     gManager.m_sceneKnownObstacles.ReadObstacles(fname);
    gManager.m_sceneKnownObstacles.AddBoundary(params.GetValueAsDouble("SceneBoundaryThickness", GP::Constants::SCENE_BOUNDARY_THICKNESS));

    gManager.m_sceneCells.m_grid.Setup2D(pin.m_gridNrDimsX, pin.m_gridNrDimsY,
 				     gManager.m_sceneKnownObstacles.GetGrid()->GetMin()[0],
				     gManager.m_sceneKnownObstacles.GetGrid()->GetMin()[1],
				     gManager.m_sceneKnownObstacles.GetGrid()->GetMax()[0],
				    gManager.m_sceneKnownObstacles.GetGrid()->GetMax()[1]);
    for(auto & cid : gManager.m_sceneKnownObstacles.m_cidsObstacles)
	gManager.m_sceneCells.AddCellObstacle(cid);
    for(int i = 0; i < gManager.m_sceneKnownObstacles.m_grid.GetNrCells(); ++i)
	if(gManager.m_sceneKnownObstacles.m_cidsObstacles.find(i) == gManager.m_sceneKnownObstacles.m_cidsObstacles.end())
	    gManager.m_sceneCells.AddCellUnknown(i);
    
    printf("sceneCells obstacles = %d unknown = %d\n", gManager.m_sceneCells.m_cidsObstacles.size(), gManager.m_sceneCells.m_cidsUnknown.size());

    
    gManager.m_decomp.m_dclearExp = pin.m_plannerClearExponent;    
    gManager.m_decomp.m_grid.Setup2D(pin.m_plannerRegionsNrDimsX, pin.m_plannerRegionsNrDimsY,
				     gManager.m_sceneKnownObstacles.GetGrid()->GetMin()[0],
				     gManager.m_sceneKnownObstacles.GetGrid()->GetMin()[1],
				     gManager.m_sceneKnownObstacles.GetGrid()->GetMax()[0],
				    gManager.m_sceneKnownObstacles.GetGrid()->GetMax()[1]);

    gManager.m_decomp.CreateRegions();
    gManager.m_dclearMax = 0.0;    
    for(int i = gManager.m_decomp.GetNrRegions() - 1; i >= 0; --i)
    {
	auto dclear = gManager.m_decomp.GetRegionByIndex(i)->m_dclear;
	if(dclear != INFINITY && dclear > gManager.m_dclearMax)
	    gManager.m_dclearMax = dclear;	
    }
    gManager.m_decomp.ReadyForMotionPlanning();
    
    
    gManager.m_gtex.SetFileName(params.GetValue("GraphicsTextureObstacles"));
    gManager.m_gtexTerrain.SetFileName("textures/terrain2.ppm");
    
    gManager.Help();
    gManager.MainLoop("GRunDecomp", 
		      params.GetValueAsInt("GraphicsResolutionX", GP::Constants::GRAPHICS_RESOLUTION_X),
		      params.GetValueAsInt("GraphicsResolutionY", GP::Constants::GRAPHICS_RESOLUTION_Y));
    
    return 0;
}
