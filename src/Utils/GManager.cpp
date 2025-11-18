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
#include "Utils/GDraw.hpp"
#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Params.hpp"
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstring>

namespace GP
{
    GManager *m_gManager = NULL;
    
    void GManager::CallbackEventOnActiveMouseMove(int x, int y)
    {
	if(m_gManager)
	{		
	    if(m_gManager->HandleEventOnActiveMouseMove(x, y))
		glutPostRedisplay();	    
	    m_gManager->m_mousePrevX = x;
	    m_gManager->m_mousePrevY = y;
	}
    }
    
    void GManager::CallbackEventOnPassiveMouseMove(int x, int y)
    {
	if(m_gManager)
	{	
	    if(m_gManager->HandleEventOnPassiveMouseMove(x, y))
		glutPostRedisplay();	    
	    m_gManager->m_mousePrevX = x;
	    m_gManager->m_mousePrevY = y;
	}
    }
    
    void GManager::CallbackEventOnMouse(int button, int state, int x, int y)
    {
	if(m_gManager)
	{
	    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN &&
	       m_gManager->HandleEventOnMouseLeftBtnDown(x, y))
		glutPostRedisplay();
	    
	    m_gManager->m_mousePrevX = x;
	    m_gManager->m_mousePrevY = y;
	}
    }
    
    void GManager::CallbackEventOnTimer(int id)
    {
	if(m_gManager)
	{
	    m_gManager->HandleEventOnTimer();
	    glutTimerFunc(m_gManager->m_timer, CallbackEventOnTimer, id);
	    glutPostRedisplay();	    
	}
    }

    void GManager::CallbackEventOnIdle(void)
    {
	if(m_gManager && m_gManager->HandleEventOnIdle())
	    glutPostRedisplay();	    
	
    }
    
    void GManager::CallbackEventOnSpecialKeyPress(int key, int x, int y)
    {
	if(m_gManager && m_gManager->HandleEventOnSpecialKeyPress(key))
	    glutPostRedisplay();
    }
    
    void GManager::CallbackEventOnNormalKeyPress(unsigned char key, int x, int y)
    {
	if(m_gManager && m_gManager->HandleEventOnNormalKeyPress(key))
	    glutPostRedisplay();
    }
    

    void GManager::CallbackEventOnSpecialKeyUp(int key, int x, int y)
    {
	if(m_gManager && m_gManager->HandleEventOnSpecialKeyUp(key))
	    glutPostRedisplay();
    }
    
    void GManager::CallbackEventOnNormalKeyUp(unsigned char key, int x, int y)
    {
	if(m_gManager && m_gManager->HandleEventOnNormalKeyUp(key))
	    glutPostRedisplay();
    }

    void GManager::SetupViews(void)
    {
	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(HasFlag(m_gManager->m_flags, FLAG_3D))
	    gluPerspective(m_gManager->m_values[INDEX_PERSPECTIVE_ANGLE],
			   (double) glutGet(GLUT_WINDOW_WIDTH) / glutGet(GLUT_WINDOW_HEIGHT),
			   m_gManager->m_values[INDEX_PERSPECTIVE_NEAR_PLANE], 
			   m_gManager->m_values[INDEX_PERSPECTIVE_FAR_PLANE]);
	else
	{
	    glOrtho(m_gManager->m_values[INDEX_MINX],
		    m_gManager->m_values[INDEX_MAXX],
		    m_gManager->m_values[INDEX_MINY],
			m_gManager->m_values[INDEX_MAXY],
			m_gManager->m_values[INDEX_ORTHO_NEAR_PLANE],
			m_gManager->m_values[INDEX_ORTHO_FAR_PLANE]);
	    }
	    
	    
	    glMatrixMode(GL_MODELVIEW);
	    glLoadIdentity();
	    
	    if(HasFlag(m_gManager->m_flags, FLAG_3D))
	    {
		double m[16];
		m_gManager->m_gCamera.GetModelViewMatrixOpenGL(m);
		glMultMatrixd(m);
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);
		glShadeModel(GL_SMOOTH);
		glEnable(GL_LIGHTING);
		GDrawIllumination(&m_gManager->m_gIllumination);
	    }
	    else
	    {
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);
		//	glPushMatrix();
		
		if(HasFlag(m_gManager->m_flags, FLAG_FLIP_VIEW))
		{
		    glTranslatef(0.0,  (m_gManager->m_values[INDEX_MINY] + m_gManager->m_values[INDEX_MAXY]), 0.0f);
		    glScalef(1.0f, -1.0f, 1.0f);
		}
		
		//glPopMatrix();
		
	    }
	    
    }

    
    void  GManager::CallbackEventOnDisplay(void)
    {
	if(m_gManager)
	{
	    
	    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	    glClearDepth(1.0);
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    

	    m_gManager->SetupViews();
	    m_gManager->HandleEventOnDisplay();
	    
	    if(HasFlag(m_gManager->m_flags, FLAG_EXPORT_FRAMES))
		ExportFrameAsImage();

	    glutSwapBuffers();	    
	}
    }
    
    GManager::GManager(void) 
    {
	m_blocking = true;
	
	m_idWindow      = -1;
	m_timer         = Constants::GRAPHICS_ONTIMER_INTERVAL;
	m_mousePrevX    = 0;
	m_mousePrevY    = 0;	    
	m_frames        = 0;
	m_flags         = AddFlag(0, /*FLAG_3D |*/ FLAG_ROTATE_X | FLAG_ROTATE_Y | FLAG_ROTATE_Z | FLAG_ROTATE_CENTER | FLAG_ROTATE_GLOBAL_AXIS|FLAG_FLIP_VIEW);

	m_values.resize(NR_INDICES);
	m_values[INDEX_CAMERA_MOVE]            = 0.5;
	m_values[INDEX_MINX]                   = -40;
	m_values[INDEX_MINY]                   = -40;
	m_values[INDEX_MAXX]                   =  40;
	m_values[INDEX_MAXY]                   =  40;
	m_values[INDEX_ORTHO_NEAR_PLANE]       = -10;
	m_values[INDEX_ORTHO_FAR_PLANE]        =  10;
	m_values[INDEX_PERSPECTIVE_ANGLE]      =  18;
	m_values[INDEX_PERSPECTIVE_NEAR_PLANE] = 100;
	m_values[INDEX_PERSPECTIVE_FAR_PLANE]  = 10000;
    }
    
    GManager::~GManager(void)
    {
	if(m_idWindow >= 0)
	    glutDestroyWindow(m_idWindow);	
    }
    
    void GManager::SetupFromParams(Params * const p)
    {
	m_values[INDEX_CAMERA_MOVE]            = p->GetValueAsDouble("GraphicsCameraMove",           m_values[INDEX_CAMERA_MOVE]);
	m_values[INDEX_MINX]                   = p->GetValueAsDouble("GraphicsMinX",                 m_values[INDEX_MINX]);
	m_values[INDEX_MINY]                   = p->GetValueAsDouble("GraphicsMinY",                 m_values[INDEX_MINY]);
	m_values[INDEX_MAXX]                   = p->GetValueAsDouble("GraphicsMaxX",                 m_values[INDEX_MAXX]);
	m_values[INDEX_MAXY]                   = p->GetValueAsDouble("GraphicsMaxY",                 m_values[INDEX_MAXY]);
	m_values[INDEX_ORTHO_NEAR_PLANE]       = p->GetValueAsDouble("GraphicsOrthoNearPlane",       m_values[INDEX_ORTHO_NEAR_PLANE]);
	m_values[INDEX_ORTHO_FAR_PLANE]        = p->GetValueAsDouble("GraphicsOrthoFarPlane",        m_values[INDEX_ORTHO_FAR_PLANE]);
	m_values[INDEX_PERSPECTIVE_ANGLE]      = p->GetValueAsDouble("GraphicsPerspectiveAngle",     m_values[INDEX_PERSPECTIVE_ANGLE]);
	m_values[INDEX_PERSPECTIVE_NEAR_PLANE] = p->GetValueAsDouble("GraphicsPerspectiveNearPlane", m_values[INDEX_PERSPECTIVE_NEAR_PLANE]);
	m_values[INDEX_PERSPECTIVE_FAR_PLANE]  = p->GetValueAsDouble("GraphicsPerspectiveFarPlane",  m_values[INDEX_PERSPECTIVE_FAR_PLANE]);
	m_timer                                = p->GetValueAsInt("OnTimerInterval", m_timer);
	
	m_gCamera.SetupFromParams(p);
	m_gIllumination.SetupFromParams(p);
    }
    
    
    void GManager::MainLoop(const char * const title, const int width, const int height)
    {	
	m_gManager = this;
	
	static int    argc = 1;	
	static char  *args = (char*)"args";
	
	glutInit(&argc, &args);    
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
	glutInitWindowSize(width, height);
	glutInitWindowPosition(0, 0); 
	m_idWindow = glutCreateWindow(title);	   	
	
	glutDisplayFunc(CallbackEventOnDisplay);
	glutMouseFunc(CallbackEventOnMouse);
	glutMotionFunc(CallbackEventOnActiveMouseMove);
	glutPassiveMotionFunc(CallbackEventOnPassiveMouseMove);
	glutIdleFunc(CallbackEventOnIdle);
	glutTimerFunc(0, CallbackEventOnTimer, 0); 
	glutKeyboardFunc(CallbackEventOnNormalKeyPress);
	glutSpecialFunc(CallbackEventOnSpecialKeyPress);
	glutKeyboardUpFunc(CallbackEventOnNormalKeyUp);
	glutSpecialUpFunc(CallbackEventOnSpecialKeyUp);

	if(m_blocking)
	    glutMainLoop();	
    }
    
    bool GManager::HandleEventOnNormalKeyPress(const int key)
    {
	if(key == 27) //escape key
	{
	    exit(0);
	    return true;
	}
	else if(glutGetModifiers() & GLUT_ACTIVE_ALT)
	{
	    if(key == 'u')
	    {
		m_flags = FlipFlag(m_flags, FLAG_FLIP_VIEW);
		return true;
	    }
	    else  if(key == 'f')
	    {
		ExportFrameAsImage();
		return true;
	    }
	    else if(key == 'm')
	    {
		m_flags = FlipFlag(m_flags, FLAG_EXPORT_FRAMES);
		return true;
	    }
	    else if(key == 'd')
	    {
		m_flags = FlipFlag(m_flags, FLAG_3D);
		return true;
	    }
	    else if(key == 'e')
	    {
		
		const double *eye    = m_gCamera.GetEye();
		const double *center = m_gCamera.GetCenter();
		const double *right  = m_gCamera.GetRightAxis();
		const double *forward= m_gCamera.GetForwardAxis();

		printf("CameraEye %f %f %f\n", eye[0], eye[1], eye[2]);
		printf("CameraCenter %f %f %f\n", center[0], center[1], center[2]);
		printf("CameraRight %f %f %f\n", right[0], right[1], right[2]);
		printf("CameraForward %f %f %f\n", forward[0], forward[1], forward[2]);
	    }	    
	    else if(key == 'x' || key == 'X' || key == 'y' || key == 'Y' || key == 'z' || key == 'Z')
	    {
		if(HasFlag(m_flags, FLAG_3D) == false)
		    return true;
		
		const double *p =  HasFlag(m_flags, FLAG_ROTATE_CENTER) ? m_gCamera.GetCenter() : m_gCamera.GetEye();
		const double theta = (key == 'x' || key == 'y' || key == 'z') ? 
		    Constants::DEG2RAD :
		   -Constants::DEG2RAD;
		
		
		if(key == 'x' || key == 'X')
		{
		    if(HasFlag(m_flags, FLAG_ROTATE_GLOBAL_AXIS))
			m_gCamera.RotateAroundAxisAtPoint(theta, 1, 0, 0, p[0], p[1], p[2]);	    
		    else
			m_gCamera.RotateAroundRightAxisAtPoint(theta, p[0], p[1], p[2]);
		}
		else if(key == 'y' || key == 'Y')
		{
		    if(HasFlag(m_flags, FLAG_ROTATE_GLOBAL_AXIS))
			m_gCamera.RotateAroundAxisAtPoint(theta, 0, 1, 0, p[0], p[1], p[2]);	    
		    else
			m_gCamera.RotateAroundUpAxisAtPoint(theta, p[0], p[1], p[2]);
		}
		else if(key == 'z' || key == 'Z')
		{
		    if(HasFlag(m_flags, FLAG_ROTATE_GLOBAL_AXIS))
			m_gCamera.RotateAroundAxisAtPoint(theta, 0, 0, 1, p[0], p[1], p[2]);	    
		    else
			m_gCamera.RotateAroundForwardAxisAtPoint(theta, p[0], p[1], p[2]);
		}
		return true;
	    }
	}
	
	return false;
    }
    
    void GManager::Help(void)
    {
	printf("Help -- Menu options\n");
	printf(" right click to see menu\n\n");	
	printf("Help -- Keyboard controls\n");	
	printf(" F1         => help\n");
	printf(" Esc        => exit\n\n");
	printf(" Alt+f      => export frame as image\n");
	printf(" Alt+m      => toggle on/off export frame as image\n");

    }
    
    
    
    bool GManager::HandleEventOnSpecialKeyPress(const int key)
    {	
	if(key == GLUT_KEY_F1)
	{
	    Help();
	    return true;
	}
	else if(HasFlag(m_flags, FLAG_3D))
	{
	    switch(key)
	    {
	    case GLUT_KEY_UP:
		m_gCamera.MoveForward(m_values[INDEX_CAMERA_MOVE]);
		return true;		
	    case GLUT_KEY_DOWN:
		m_gCamera.MoveForward(-m_values[INDEX_CAMERA_MOVE]);
		return true;		
	    case GLUT_KEY_RIGHT:
	   	m_gCamera.MoveRight(m_values[INDEX_CAMERA_MOVE]);
		return true;		
	    case GLUT_KEY_LEFT:
	   	m_gCamera.MoveRight(-m_values[INDEX_CAMERA_MOVE]);
		return true;		
	    case GLUT_KEY_PAGE_DOWN:
		m_gCamera.MoveUp(m_values[INDEX_CAMERA_MOVE]);
		return true;	
	    case GLUT_KEY_PAGE_UP:
		m_gCamera.MoveUp(-m_values[INDEX_CAMERA_MOVE]);
		return true;
	    }
	}
	
	return false;
    }
    
    bool GManager::HandleEventOnActiveMouseMove(const int x, const int y)
    {
	const double thetax = 0.25 * 2 * M_PI * (y - m_mousePrevY) / glutGet(GLUT_WINDOW_HEIGHT);
	const double thetay = 0.25 * 2 * M_PI * (x - m_mousePrevX) / glutGet(GLUT_WINDOW_WIDTH);	    
	const double thetaz = thetay;
	bool         event  = false;	
	
	const double *p =  HasFlag(m_flags, FLAG_ROTATE_CENTER) ?  m_gCamera.GetCenter() : m_gCamera.GetEye();
	
	if(HasFlag(m_flags, FLAG_ROTATE_X))
	{
	    if(HasFlag(m_flags, FLAG_ROTATE_GLOBAL_AXIS))
		m_gCamera.RotateAroundAxisAtPoint(thetax, 1, 0, 0, p[0], p[1], p[2]);	    
	    else
		m_gCamera.RotateAroundRightAxisAtPoint(thetax, p[0], p[1], p[2]);
	    event = true;	    
	}
	
	if(HasFlag(m_flags, FLAG_ROTATE_Y))
	{
	    if(HasFlag(m_flags, FLAG_ROTATE_GLOBAL_AXIS))
		m_gCamera.RotateAroundAxisAtPoint(thetay, 0, 1, 0, p[0], p[1], p[2]);	    
	    else
		m_gCamera.RotateAroundUpAxisAtPoint(thetay, p[0], p[1], p[2]);
	    event = true;	    
	}
	

	if(HasFlag(m_flags, FLAG_ROTATE_Z))
	{
	    if(HasFlag(m_flags, FLAG_ROTATE_GLOBAL_AXIS))
		m_gCamera.RotateAroundAxisAtPoint(thetaz, 0, 0, 1, p[0], p[1], p[2]);	    
	    else
		m_gCamera.RotateAroundForwardAxisAtPoint(thetaz, p[0], p[1], p[2]);
	    event = true;	    
	}
	
	return event;
    }
    
 
    void GManager::MousePosFromScreenToWorld(const int x, const int y, 
					     double * const wx,
					     double * const wy,
					     double * const wz)
    {
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ=0;
	double px = 0, py = 0, pz = 0;
	
	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );
	
	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &px, &py, &pz);
	
	if(wx)
	    *wx = px;
	if(wy)
	    *wy = py;
	if(wz)
	    *wz = pz;    
    }
    
    void GManager::ExportFrameAsImage(void)
    {
	int  ignore;
	char fname[50];
	sprintf(fname, "frames_%05d.ppm", (m_gManager->m_frames)++);		    
	ExportFrameAsImage(fname);

	char cmd[300];
	sprintf(cmd, "convert -quality 100 %s frames_%05d.jpg", fname, (m_gManager->m_frames));
	ignore = system(cmd);
	sprintf(cmd, "\\rm %s", fname);
	ignore = system(cmd);
	
    }
    
    void GManager::ExportFrameAsImage(const char fname[])
    {
	
	const int width = glutGet(GLUT_WINDOW_WIDTH);
	const int height= glutGet(GLUT_WINDOW_HEIGHT);
	
	char *temp  = new char[3 * width * height];
	char *image = new char[3 * width * height];
	
	FILE *fp = fopen(fname, "w");
	
	printf("Writing %s\n", fname);
	
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, temp);
	
	int  a, b, row_sz = 3*width;
	// Reverse rows
	for(int i=0; i < height; i+=1) 
	{
	    for(int j=0; j < width; j+=1) 
	    {
		a = i*row_sz+3*j;
		b = (height-i-1)*row_sz+3*j;
		image[a]   = temp[b];
		image[a+1] = temp[b+1];
		image[a+2] = temp[b+2];
	    }
	}
	fprintf(fp, "P6\n");
	fprintf(fp, "%i %i\n 255\n", width, height);
	fwrite(image, sizeof(char), 3 * width * height, fp);
	fclose(fp);	    
	delete[] temp;
	delete[] image;
    }

   
}



