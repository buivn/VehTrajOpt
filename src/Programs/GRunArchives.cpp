/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */
#include "Utils/GManager.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include "Utils/GTexture.hpp"
#include "GP/ScenePolys.hpp"
#include "Programs/Archives.hpp"
#include "GP/MPSnakeSimulator.hpp"
#include <fstream>
#include <iostream>
#include "TrajOpt/DevPlanner.hpp"
#include "GP/Setup.hpp"

namespace GP
{	
    class GRunArchivesManager : public GManager
    {
    public:
	enum
	    {
		FLAG_PAUSE = 1,
		FLAG_DRAW_STATE = 2,
		FLAG_DRAW_SCENE = 4,
		FLAG_DRAW_GOAL = 8,
		FLAG_DRAW_FREE_CELLS = 16,
		FLAG_DRAW_OBSTACLE_CELLS = 32,
		FLAG_DRAW_TRAJ = 64,
		FLAG_DRAW_REMAINING_PATH = 128		
	    };
	
	GRunArchivesManager(void) : GManager()
	{
	    m_flags  = AddFlag(0,
			       FLAG_PAUSE|
			       FLAG_DRAW_STATE |
			       FLAG_DRAW_SCENE |
			       FLAG_DRAW_GOAL |
			       FLAG_DRAW_FREE_CELLS |
			       FLAG_DRAW_OBSTACLE_CELLS|
			       FLAG_DRAW_TRAJ|
			       FLAG_DRAW_REMAINING_PATH);
	    m_archivesPos = 0;

	    m_gresX2 = m_gresY2 = 0.0;
	    
	}
	
	virtual ~GRunArchivesManager(void)
	{
	}

	
	virtual void SetupViews(void)
	{
	    const double bigw = glutGet(GLUT_WINDOW_WIDTH);
	    const double bigh= glutGet(GLUT_WINDOW_HEIGHT);	    
	    const double smallw = m_gresX2;
	    const double smallh  = m_gresY2;
	    const double w1width = bigw - smallw;
	    const double w1height = bigh;
	    const double w2width = smallw;
	    const double w2height= smallh;
	    const double *pmin = m_sceneKnownObstacles.GetGrid()->GetMin();
	    const double *pmax =m_sceneKnownObstacles.GetGrid()->GetMax();
	    const double off = 0.002;
	    
	    
	    glViewport(0, 0,  w1width, w1height);
	    
	    SetValue(INDEX_MINX, pmin[0] - off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MINY, pmin[1] - off * (pmax[1] - pmin[1]));
	    SetValue(INDEX_MAXX, pmax[0] + off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MAXY, pmax[1] + off * (pmax[1] - pmin[1]));
	    
	    glMatrixMode(GL_PROJECTION);
	    glLoadIdentity();
	    if(HasFlag(GManager::m_flags, FLAG_3D))
			gluPerspective(m_values[INDEX_PERSPECTIVE_ANGLE],
					(double) w1width / w1height,
					m_values[INDEX_PERSPECTIVE_NEAR_PLANE], 
					m_values[INDEX_PERSPECTIVE_FAR_PLANE]);
	    else
	    {
			glOrtho(m_values[INDEX_MINX],
				m_values[INDEX_MAXX],
				m_values[INDEX_MINY],
				m_values[INDEX_MAXY],
				m_values[INDEX_ORTHO_NEAR_PLANE],
				m_values[INDEX_ORTHO_FAR_PLANE]);
	    }

	    glMatrixMode(GL_MODELVIEW);
	    glLoadIdentity();
	    
	    if(HasFlag(GManager::m_flags, FLAG_3D))
	    {
			double m[16];
			m_gCamera.GetModelViewMatrixOpenGL(m);
			glMultMatrixd(m);
			glEnable(GL_TEXTURE_2D);
			glEnable(GL_DEPTH_TEST);
			glShadeModel(GL_SMOOTH);
			glEnable(GL_LIGHTING);
			GDrawIllumination(&m_gIllumination);
	    }
	    else
	    {
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			//	glPushMatrix();
			if(HasFlag(GManager::m_flags, FLAG_FLIP_VIEW))
			{
				glTranslatef(0.0,  (m_values[INDEX_MINY] + m_values[INDEX_MAXY]), 0.0f);
				glScalef(1.0f, -1.0f, 1.0f);
			}
			//glPopMatrix();
	    }
	    DrawFn(pmin, pmax);

	    ///////////// WINDOW 2
	    if(m_gresX2 <= 0 || m_gresY2 <= 0)
		return;
	    
	    glViewport(w1width, (w1height - w2height) * 0.5,  w2width, w2height);

	       	    double bmin[2];
	    double bmax[2];
	    ArchiveEntry *ae = m_archivesPos < m_archives.m_entries.size() ? m_archives.m_entries[m_archivesPos] : NULL;	    

	    if(ae == NULL)
		return;

	    const double x = ae->m_state[0];
	    const double y = ae->m_state[1];
	    const double r = 50;

	    bmin[0] = x - r;
	    bmin[1] = y - r;
	    bmax[0] = x + r;
	    bmax[1] = y + r;

	    SetValue(INDEX_MINX, bmin[0] - off * (bmax[0] - bmin[0]));
	    SetValue(INDEX_MINY, bmin[1] - off * (bmax[1] - bmin[1]));
	    SetValue(INDEX_MAXX, bmax[0] + off * (bmax[0] - bmin[0]));
	    SetValue(INDEX_MAXY, bmax[1] + off * (bmax[1] - bmin[1]));
	    
	    glMatrixMode(GL_PROJECTION);
	    glLoadIdentity();

	    if(HasFlag(GManager::m_flags, FLAG_3D))
		gluPerspective(m_values[INDEX_PERSPECTIVE_ANGLE],
			       (double) w1width / w1height,
			       m_values[INDEX_PERSPECTIVE_NEAR_PLANE], 
			       m_values[INDEX_PERSPECTIVE_FAR_PLANE]);
	    else
	    {
		glOrtho(m_values[INDEX_MINX],
			m_values[INDEX_MAXX],
			m_values[INDEX_MINY],
			m_values[INDEX_MAXY],
			m_values[INDEX_ORTHO_NEAR_PLANE],
			m_values[INDEX_ORTHO_FAR_PLANE]);
	    }

	    glMatrixMode(GL_MODELVIEW);
	    glLoadIdentity();
	    
	    if(HasFlag(GManager::m_flags, FLAG_3D))
	    {
		double m[16];
		m_gCamera.GetModelViewMatrixOpenGL(m);
		glMultMatrixd(m);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);
		glShadeModel(GL_SMOOTH);
		glEnable(GL_LIGHTING);
		GDrawIllumination(&m_gIllumination);
	    }
	    else
	    {
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			//	glPushMatrix();
			
			if(HasFlag(GManager::m_flags, FLAG_FLIP_VIEW))
			{
				glTranslatef(0.0,  (m_values[INDEX_MINY] + m_values[INDEX_MAXY]), 0.0f);
				glScalef(1.0f, -1.0f, 1.0f);
			}
			//glPopMatrix();
	    }
	    DrawFn(bmin, bmax);
	}
	
	virtual void DrawFn(const double pmin[], const double pmax[])
	{
	    const bool    is2D = GDrawIs2D();
	    const double  off  = 0.002;
	    ArchiveEntry *ae = m_archivesPos < m_archives.m_entries.size() ? m_archives.m_entries[m_archivesPos] : NULL;	    
	    double        rgb[3];
	    double bboxCell[4];	    
	    GMaterial     gmat;
	    Polygon2D poly;
	    TriMesh tmesh;
	    
	    m_gtex.AutomaticCoords();
	    m_gtex.Use();
	    gmat.SetTurquoise();
	    gmat.SetDiffuse(0.0,0.0, 0.0);
	    
	    GDraw2D();
	    SetValue(INDEX_MINX, pmin[0] - off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MINY, pmin[1] - off * (pmax[1] - pmin[1]));
	    SetValue(INDEX_MAXX, pmax[0] + off * (pmax[0] - pmin[0]));
	    SetValue(INDEX_MAXY, pmax[1] + off * (pmax[1] - pmin[1]));

	    if(HasFlag(m_flags, FLAG_DRAW_SCENE))
	    {
			if(HasFlag(GManager::m_flags, FLAG_3D) == false)
			{
				GDraw2D();
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
	    
	    
	    GDrawWireframe(false);
	    GDrawLineWidth(1.0);

	
	    

	    
	    // if(HasFlag(m_flags, FLAG_DRAW_OBSTACLE_CELLS) && ae)
	    // {
		// 	if(HasFlag(GManager::m_flags, FLAG_3D) == false)
		// 	{
		// 		GDraw2D();
		// 		GDrawColor(0.0, 0.0, 0.0);
		// 		for(auto & cid : ae->m_cidsObstacles)
		// 		{
		// 		m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
		// 		GDrawAABox2D(bboxCell[0], bboxCell[1],  bboxCell[2], bboxCell[3]);		
		// 		}
		// 	}
		// 	else
		// 	{
		// 		GDraw3D();
		// 		gmat.SetAmbient(0.0, 0.0, 0.0);
		// 		GDrawMaterial(&gmat);		    
		// 		for(auto & cid : ae->m_cidsObstacles)
		// 		{
		// 		m_sceneKnownObstacles.GetGrid()->GetCellFromId(cid, bboxCell);
		// 		GDrawBox3D(bboxCell[0], bboxCell[1],  0.0, bboxCell[2], bboxCell[3], 1.1);		
		// 		}	
		// 	}
		
	    // }
	    
	    if(HasFlag(m_flags, FLAG_DRAW_TRAJ) && ae)
	    {
		    GDraw2D();
		    GDrawColor(1, 0, 1);
		    GDrawLineWidth(8.0);
		    GDrawPushTransformation();
		    GDrawMultTrans(0, 0, 0.03);
		    
		    GDrawColor(1, 0, 1);
		    for(int i = 2; i < ae->m_trajPoints.size();  i += 2)
			GDrawSegment2D(&(ae->m_trajPoints[i-2]), &(ae->m_trajPoints[i]));
		    GDrawPopTransformation();
	    }
	    
	    if(HasFlag(m_flags, FLAG_DRAW_REMAINING_PATH) && ae)
	    {
		GDraw2D();
		GDrawColor(1, 0, 1);
		GDrawLineWidth(8.0);
		GDrawPushTransformation();
		GDrawMultTrans(0, 0, 0.03);
		
		GDrawColor(0, 1, 1);
		for(int i = 2; i < ae->m_remainingPathPoints.size();  i += 2)
		    GDrawSegment2D(&(ae->m_remainingPathPoints[i-2]), &(ae->m_remainingPathPoints[i]));
		GDrawPopTransformation();		
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_GOAL)  && ae)
	    {
			GDraw2D();
			GDrawColor(0, 0, 1);
			GDrawLineWidth(4.0);
			//GDrawWireframe(true);
			GDrawPushTransformation();
			GDrawMultTrans(0, 0, 0.1);
			GDrawCircle2D(ae->m_goalX, ae->m_goalY, ae->m_goalRadius);
			GDrawPopTransformation();
			GDrawWireframe(false);
			GDrawLineWidth(2.0);
	    }

	    if(HasFlag(m_flags, FLAG_DRAW_STATE) && ae)
	    {
			//other way of drawing
			PlannerInput pin;
			auto snake = dynamic_cast<MPSnakeSimulator*>(&(m_sim));
			std::vector<double> bodies;
			pin.m_snakeNrLinks = snake->m_nrLinks;
			pin.m_snakeLinkWidth = snake->m_bodyWidth;
			pin.m_snakeLinkLength = snake->m_bodyLength;
			pin.m_snakeAttachDistance = snake->m_attachDist;
			bodies.resize(8 * (1 + pin.m_snakeNrLinks));		
			// SnakePlacement(pin, &(ae->m_state[0]), &bodies[0]); Check here later ----------
			if(HasFlag(GManager::m_flags, FLAG_3D) == false)
			{
				GDrawColor(1, 0, 0);		
				for(int i = 0; i < bodies.size(); i += 8)
				{
				if(i == 0)
					GDrawColor(1, 0, 0);
				else
					GDrawColor(0, 0, 1);
				GDrawQuad2D(&bodies[i]);
				}		    
			}
			else
			{
				GDraw3D();
				for(int i = 0; i < bodies.size(); i += 8)
				{
				if(i == 0)
					gmat.SetAmbient(1, 0, 0);
				else
					gmat.SetAmbient(0.0, 0.0, 0.7);
				poly.Clear();
				poly.AddVertices(4, &bodies[i]);
				tmesh.Clear();
				tmesh.AddExtrudedPolygon(&poly, 0.0, 0.75);
				GDrawMaterial(&gmat);
				tmesh.Draw();
				}	
				GDraw2D();
			}
	    }  
	}
	
	
	virtual void HandleEventOnDisplay(void)
	{
	    
	    GManager::HandleEventOnDisplay();

	    return;
	    
	    const double *pmin = m_sceneKnownObstacles.GetGrid()->GetMin();
	    const double *pmax =m_sceneKnownObstacles.GetGrid()->GetMax();

	    DrawFn(pmin, pmax);

	    double bmin[2];
	    double bmax[2];
	    ArchiveEntry *ae = m_archivesPos < m_archives.m_entries.size() ? m_archives.m_entries[m_archivesPos] : NULL;	    

	    if(ae == NULL)
			return;

	    const double x = ae->m_state[0];
	    const double y = ae->m_state[1];
	    const double r = 50;

	    bmin[0] = x - 0.5 * r;
	    bmin[1] = y - 0.5 * r;
	    bmax[0] = x + 0.5 * r;
	    bmax[1] = y + 0.5 * r;

	    DrawFn(bmin, bmax);
	    
	    
	}	
	
	virtual bool HandleEventOnNormalKeyPress(const int key)
	{
	    if(key == 'p')
		m_flags = FlipFlag(m_flags, FLAG_PAUSE);
	    else if(key == '0')
		m_archivesPos = 0;
	    
	    return GManager::HandleEventOnNormalKeyPress(key);
	}

	virtual void HandleEventOnTimer(void)
	{
	    if(HasFlag(m_flags, FLAG_PAUSE))
		return;
	    if(m_archivesPos + 1 < m_archives.m_entries.size())
		++m_archivesPos;
	}

	MPSnakeSimulator m_sim;	
	ScenePolys m_sceneKnownObstacles;
	Archives m_archives;
	int m_archivesPos;
	
	Flags            m_flags;
	GTexture         m_gtex;
	GTexture  m_gtexTerrain;

	double m_gresX2;
	double m_gresY2;
	
    };
};


extern "C" int GRunArchives(int argc, char **argv)
{
    GP::GRunArchivesManager gManager;
    GP::Params params;
    GP::PlannerInput pin;
    
    printf("usage: ./bin/Runner GRunArchives fnameParams\n");
    
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

    //set up second window (if any)
    gManager.m_gresX2 = params.GetValueAsDouble("GraphicsResolutionX2", 0);
    gManager.m_gresY2 = params.GetValueAsDouble("GraphicsResolutionY2", 0);
    
    //setup scene with known obstacles
    gManager.m_sceneKnownObstacles.SetupFromParams(&params);
    auto fname = params.GetValue("SceneOccupancyFile", NULL);
    if(fname)
	gManager.m_sceneKnownObstacles.ReadOccupancyGrid(fname);
    else if((fname = params.GetValue("SceneObstaclesFile", NULL)))
	gManager.m_sceneKnownObstacles.ReadObstacles(fname);
    gManager.m_sceneKnownObstacles.AddBoundary(params.GetValueAsDouble("SceneBoundaryThickness", GP::Constants::SCENE_BOUNDARY_THICKNESS));
    
    //setup simulator
    gManager.m_sim.SetupFromParams(&params);
    gManager.m_sim.CompleteSetup();
   
   //read archives
   fname = params.GetValue("ArchivesFile", "data/archives.txt");
   if(fname)
   {
       std::ifstream in(fname);
       gManager.m_archives.Read(in);       
       in.close();
   }
   
    
    gManager.m_gtex.SetFileName(params.GetValue("GraphicsTextureObstacles"));
    gManager.m_gtexTerrain.SetFileName("textures/terrain2.ppm");
    
    gManager.MainLoop("GRunArchives", 
		      params.GetValueAsInt("GraphicsResolutionX", GP::Constants::GRAPHICS_RESOLUTION_X),
		      params.GetValueAsInt("GraphicsResolutionY", GP::Constants::GRAPHICS_RESOLUTION_Y));
    
    return 0;
}
