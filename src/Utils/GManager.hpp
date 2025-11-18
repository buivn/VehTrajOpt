/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef  GP__GMANAGER_HPP_
#define  GP__GMANAGER_HPP_

#include "Utils/GDraw.hpp"
#include "Utils/GCamera.hpp"
#include "Utils/GIllumination.hpp"
#include "Utils/Flags.hpp"
#include "Utils/Params.hpp"
#include <vector>

namespace GP
{
    class GManager
    {
    public:
	GManager(void);
	
	virtual ~GManager(void);

	virtual void SetupFromParams(Params * const p);
	
	virtual void Help(void);
	
	virtual void MainLoop(const char * const title, 
			      const int width = 1000, 
			      const int height=  600);
	
	static void MousePosFromScreenToWorld(const int x, const int y, 
					      double * const wx,
					      double * const wy,
					      double * const wz);
	
	static void ExportFrameAsImage(void);
	
	static void ExportFrameAsImage(const char fname[]);
	
	
	virtual void HandleEventOnTimer(void)
	{
	}
	
	virtual void HandleEventOnDisplay(void)
	{
	}
	
	virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y)
	{
	    return false;
	}
	
	virtual bool HandleEventOnActiveMouseMove(const int x, const int y);
		
	virtual bool HandleEventOnPassiveMouseMove(const int x, const int y)
	{
	    return false;
	}
	
	virtual bool HandleEventOnNormalKeyPress(const int key);

	virtual bool HandleEventOnSpecialKeyPress(const int key);

	virtual bool HandleEventOnNormalKeyUp(const int key)
	{
	    return false;
	}
	
	virtual bool HandleEventOnSpecialKeyUp(const int key)
	{
	    return false;
	}
	
	virtual bool HandleEventOnIdle(void)
	{
	    return false;
	}

	virtual double GetValue(const int i)
	{
	    return m_values[i];
	}
	
	virtual void SetValue(const int i, const double val)
	{
	    m_values[i] = val;
	}

	bool m_blocking;
	

	virtual void SetupViews(void);

	
	//protected:	
	static void CallbackEventOnActiveMouseMove(int x, int y);
	static void CallbackEventOnPassiveMouseMove(int x, int y);
	static void CallbackEventOnMouse(int button, int state, int x, int y);
	static void CallbackEventOnTimer(int id);
	static void CallbackEventOnMenu(int item);
	static void CallbackEventOnSpecialKeyPress(int key, int x, int y);
	static void CallbackEventOnNormalKeyPress(unsigned char key, int x, int y);
	static void CallbackEventOnSpecialKeyUp(int key, int x, int y);
	static void CallbackEventOnNormalKeyUp(unsigned char key, int x, int y);

	static void CallbackEventOnDisplay(void);
	static void CallbackEventOnIdle(void);

	int           m_idWindow;
	int           m_timer;
	int           m_mousePrevX;
	int           m_mousePrevY;
	int           m_frames;
	GCamera       m_gCamera;
	GIllumination m_gIllumination;
	Flags         m_flags;

	enum
	    {
		FLAG_EXPORT_FRAMES      = 1 << 0,
		FLAG_3D                 = 1 << 1,
		FLAG_ROTATE_CENTER      = 1 << 2,
		FLAG_ROTATE_GLOBAL_AXIS = 1 << 3,
		FLAG_ROTATE_X           = 1 << 4,
		FLAG_ROTATE_Y           = 1 << 5,
		FLAG_ROTATE_Z           = 1 << 6,
		FLAG_FLIP_VIEW = 1 << 7
	    };

	enum
	    {
		INDEX_CAMERA_MOVE = 0,
		INDEX_MINX,
		INDEX_MINY,
		INDEX_MAXX,
		INDEX_MAXY,
		INDEX_ORTHO_NEAR_PLANE,
		INDEX_ORTHO_FAR_PLANE,
		INDEX_PERSPECTIVE_NEAR_PLANE,
		INDEX_PERSPECTIVE_FAR_PLANE,
		INDEX_PERSPECTIVE_ANGLE,
		NR_INDICES
	    };
	
	    

	std::vector<double> m_values;
    };
}

#endif
