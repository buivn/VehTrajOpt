/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "GP/MPSnakeSimulator.hpp"
#include "GP/GPConstants.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/GDraw.hpp"

namespace GP
{   
    MPSnakeSimulator::MPSnakeSimulator(void) : MPStandardSimulator()
    {
	m_frontWheelDriving = Constants::ROBOT_FRONT_WHEEL_DRIVING;
	m_bodyLength        = Constants::ROBOT_BODY_LENGTH;
	m_bodyWidth         = Constants::ROBOT_BODY_WIDTH;
	m_nrLinks           = Constants::SNAKE_NR_LINKS;
	m_attachDist        = Constants::SNAKE_ATTACH_DISTANCE;
	m_minSteerAngle     = Constants::ROBOT_MIN_STEER_ANGLE;
	m_maxSteerAngle     = Constants::ROBOT_MAX_STEER_ANGLE;
	m_minVel            = Constants::ROBOT_MIN_VELOCITY;
	m_maxVel            = Constants::ROBOT_MAX_VELOCITY;
	m_minAcc            = Constants::ROBOT_MIN_ACCELERATION;
	m_maxAcc            = Constants::ROBOT_MAX_ACCELERATION;
	m_minSteerVel       = Constants::ROBOT_MIN_STEER_VELOCITY;
	m_maxSteerVel       = Constants::ROBOT_MAX_STEER_VELOCITY;

	m_allocatorControl.SetDim(2);
	//m_car.AdjustDimensions(1, m_bodyWidth);
    }

    void MPSnakeSimulator::SetupFromParams(Params * const p)
    {
	MPSimulator::SetupFromParams(p);
	
	m_frontWheelDriving = p->GetValueAsInt("RobotFrontWheelDriving", m_frontWheelDriving);	    
	m_bodyLength        = p->GetValueAsDouble("SnakeLinkLength", m_bodyLength);
	m_bodyWidth         = p->GetValueAsDouble("SnakeLinkWidth",  m_bodyWidth);
	m_nrLinks           = p->GetValueAsInt("SnakeNrLinks", m_nrLinks);
	m_attachDist        = p->GetValueAsDouble("SnakeAttachDistance", m_attachDist);
	m_minSteerAngle     = Constants::DEG2RAD * p->GetValueAsDouble("RobotMinSteerAngleInDegrees", m_minSteerAngle * Constants::RAD2DEG);
	m_maxSteerAngle     = Constants::DEG2RAD * p->GetValueAsDouble("RobotMaxSteerAngleInDegrees", m_maxSteerAngle * Constants::RAD2DEG);
	m_minVel            = p->GetValueAsDouble("RobotMinVelocity", m_minVel);
	m_maxVel            = p->GetValueAsDouble("RobotMaxVelocity", m_maxVel);
	m_minAcc            = p->GetValueAsDouble("RobotMinAcceleration", m_minAcc);
	m_maxAcc            = p->GetValueAsDouble("RobotMaxAcceleration", m_maxAcc);
	m_minSteerVel       = p->GetValueAsDouble("RobotMinSteerVelocity", m_minSteerVel);
	m_maxSteerVel       = p->GetValueAsDouble("RobotMaxSteerVelocity", m_maxSteerVel);

//	m_car.AdjustDimensions(1, m_bodyWidth);
    
    }
    
    
    void MPSnakeSimulator::CompleteSetup(void)
    {	
	m_allocatorState.SetDim(5 + m_nrLinks);
	
	MPStandardSimulator::CompleteSetup();
	
	const double hw = 0.5 * m_bodyWidth;
	
	m_orig[0] = 0;             m_orig[1] = -hw;
	m_orig[2] = m_bodyLength;  m_orig[3] = -hw;	
	m_orig[4] = m_bodyLength;  m_orig[5] =  hw;	
	m_orig[6] = 0;             m_orig[7] =  hw;	
	
	//printf("..using snake model: length = %f, width = %f nrLinks = %d\n", m_bodyLength, m_bodyWidth, m_nrLinks);
	
	
	m_bodies.resize(8 * (m_nrLinks + 1));
	m_TRs.resize(Algebra2D::TransRot_NR_ENTRIES * (1 + m_nrLinks));

	SetState(m_currState);
	
    }
    
    void MPSnakeSimulator::ConstructTriMesh(void)
    {
	double quad3[12];
	
	if(m_tmeshSnake.GetNrVertices() != 0)
	    return;
	for(int i = 0; i < (int) m_bodies.size(); i += 8)
	{
	    quad3[0] = m_bodies[i];
	    quad3[1] = m_bodies[i + 1];
	    quad3[2] = 0.0;
	    quad3[3] = m_bodies[i + 2];
	    quad3[4] = m_bodies[i + 3];
	    quad3[5] = 0.0;
	    quad3[6] = m_bodies[i + 4];
	    quad3[7] = m_bodies[i + 5];
	    quad3[8] = 0.0;
	    quad3[9] = m_bodies[i + 6];
	    quad3[10] = m_bodies[i + 7];
	    quad3[11] = 0.0;
	    m_tmeshSnake.AddQuad(quad3, true);
	}
	
    }
    
    void MPSnakeSimulator::SetState(const double s[])
    {
	m_tmeshSnake.Clear();
	
	MPStandardSimulator::SetState(s);
	
	m_currState[STATE_STEER_ANGLE] = Algebra2D::AngleNormalize(m_currState[STATE_STEER_ANGLE], -M_PI);
 	m_currState[STATE_THETA] = Algebra2D::AngleNormalize(m_currState[STATE_THETA], -M_PI);
 
	
	double TR[Algebra2D::TransRot_NR_ENTRIES];
	double T[Algebra2D::Trans_NR_ENTRIES];
	double p[2];
	
	TR[0] = p[0] = s[STATE_X];
	TR[1] = p[1] = s[STATE_Y];
	Algebra2D::AngleAsRot(s[STATE_THETA], &TR[Algebra2D::Trans_NR_ENTRIES]);
	ApplyTransRotToPolygon2D(TR, 4, m_orig, &m_bodies[0]);
	
	Algebra2D::TransRotAsTransRot(TR, &m_TRs[0]);
	
	for(int i = 0; i < m_nrLinks; ++i)
	{
	    T[0] = p[0] - (m_attachDist + m_bodyLength);
	    T[1] = p[1];
	    Algebra2D::RotateAroundPointAsTransRot(s[STATE_LINKS + i], p, TR);
	    Algebra2D::TransRotMultTransAsTransRot(TR, T, TR);
	    ApplyTransRotToPolygon2D(TR, 4, m_orig, &m_bodies[8 * (i + 1)]);
	    Algebra2D::TransRotAsTransRot(TR, &m_TRs[(i + 1) * Algebra2D::TransRot_NR_ENTRIES]);
	    p[0] = TR[0];
	    p[1] = TR[1];
	}
    }
    
    
    void MPSnakeSimulator::MotionEqs(const double s[],
				     const double t,
				     const double u[],
				     double       ds[]) 
    {
	
	const double steer = s[STATE_STEER_ANGLE];
	
	if(m_frontWheelDriving)
	{
	    const double cpsi = cos(steer);
	    ds[STATE_X]       = s[STATE_VEL] * cos(s[STATE_THETA]) * cpsi;
	    ds[STATE_Y]       = s[STATE_VEL] * sin(s[STATE_THETA]) * cpsi;
	    ds[STATE_THETA]   = s[STATE_VEL] * sin(steer) / m_bodyLength;
	}
	else
	{		
	    ds[STATE_X]     = s[STATE_VEL] * cos(s[STATE_THETA]);
	    ds[STATE_Y]     = s[STATE_VEL] * sin(s[STATE_THETA]);
	    ds[STATE_THETA] = s[STATE_VEL] * tan(steer) / m_bodyLength;
	}
	
	ds[STATE_VEL]       = u[CONTROL_ACC];
	ds[STATE_STEER_ANGLE] = u[CONTROL_STEER_VEL];	    
	
	double coeff = s[STATE_VEL] / (m_bodyLength + m_attachDist);
	double prev  = s[STATE_THETA];
	
	for(int i = 0; i < m_nrLinks; ++i)
	{
	    ds[STATE_LINKS + i] = coeff * sin(prev - s[STATE_LINKS + i]);
	    coeff = coeff * cos(prev - s[STATE_LINKS + i]);
	    prev  = s[STATE_LINKS + i];
	}
    }

    bool MPSnakeSimulator::IsStateValid(void)
    {
	const double  asteer = Algebra2D::AngleNormalize(m_currState[STATE_STEER_ANGLE], -M_PI);
	const double *pmin   = GetScene()->GetGrid()->GetMin();
	const double *pmax   = GetScene()->GetGrid()->GetMax();
	
//values within limits
	
	if(asteer < m_minSteerAngle    || asteer > m_maxSteerAngle    ||
	   // m_currState[STATE_VEL] < m_minVel || m_currState[STATE_VEL] > m_maxVel ||
	   m_currState[STATE_X] < pmin[0]    || m_currState[STATE_X] > pmax[0]    ||
	   m_currState[STATE_Y] < pmin[1]    || m_currState[STATE_Y] > pmax[1])
	    return false;

//collision with obstacles	
	for(int i = 0; i < m_bodies.size(); i += 8)
	    if(m_scene->IsQuadInCollision(&m_bodies[i]))
		return false;
	
//self collision
	const int n = m_bodies.size();
	for(int i = 0; i < n; i += 8)
	    for(int j = i + 32; j < n; j += 8)
		if(CollisionConvexPolygons2D(4, &m_bodies[i], 4, &m_bodies[j]))
		    return false;
	
	return true;
    }

    void MPSnakeSimulator::StartSteerToPosition(const double target[])
    {
	m_pidSteer.Reset();
	m_pidVel.Reset();
	
	m_pidSteer.SetDesiredValue(0);
	m_pidVel.SetDesiredValue(RandomUniformReal(0.6, 0.8) * m_maxVel);
    }
    
    void MPSnakeSimulator::SteerToPosition(const double target[])
    {
	const double u[2] = {target[0] - m_currState[STATE_X], target[1] - m_currState[STATE_Y]};
	const double v[2] = {cos(m_currState[STATE_THETA] + m_currState[STATE_STEER_ANGLE]), 
			     sin(m_currState[STATE_THETA] + m_currState[STATE_STEER_ANGLE])};
	
	m_currControl[CONTROL_STEER_VEL] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);
	if(m_currControl[CONTROL_STEER_VEL] > m_maxSteerVel)
	    m_currControl[CONTROL_STEER_VEL] = m_maxSteerVel;
	else if(m_currControl[CONTROL_STEER_VEL] < m_minSteerVel)
	    m_currControl[CONTROL_STEER_VEL] = m_minSteerVel;
	
	m_currControl[CONTROL_ACC] = m_pidVel.Update(m_currState[STATE_VEL], m_dt);
	if(m_currControl[CONTROL_ACC] > m_maxAcc)
	    m_currControl[CONTROL_ACC] = m_maxAcc;
	else if(m_currControl[CONTROL_ACC] < m_minAcc)
	    m_currControl[CONTROL_ACC] = m_minAcc;
    }
    
    bool MPSnakeSimulator::HasReachedSteerPosition(const double target[], const double dtol)
    {
	return Algebra2D::PointDistSquared(target, m_currState) <= dtol * dtol;
    }
    
    void MPSnakeSimulator::Draw(void)
    {
	double        T3[Algebra3D::Trans_NR_ENTRIES];
	double        R3[Algebra3D::Rot_NR_ENTRIES];
	const double *TR;
	GMaterial     gmat;

	MPStandardSimulator::Draw();
		
	if(GDrawIs2D())
	{
	    GDrawColor(1, 0, 0);
	    GDrawQuad2D(&m_bodies[0]);
	    GDrawColor(0, 1, 0);
	    for(int i = 8; i < (int) m_bodies.size(); i += 8)
		GDrawQuad2D(&m_bodies[i]);
	}
	else
	{
	    	    
	    gmat.SetPearl();

	    Algebra3D::IdentityAsRot(R3);
	    T3[2] = 0.0;
	    
	    for(int i = 0; i <= m_nrLinks; ++i)
	    {
		TR = &m_TRs[i * Algebra2D::TransRot_NR_ENTRIES];
		
		T3[0] = TR[0];
		T3[1] = TR[1];
		
		R3[0] = R3[4] = TR[2];
		R3[3] = TR[3];
		R3[1] = -R3[3];
	
		if(i == 0)
		    gmat.SetDiffuse(0.0, 0.0, 0.0);
		else
		    gmat.SetDiffuse(0.0, 1, 0.0);
		GDrawMaterial(&gmat);
		GDrawPushTransformation();
		GDrawMultTransRot(T3, R3);
		GDrawMultTrans(0, 0, 0.5 * m_bodyWidth);
		GDrawMultFromZAxisToAxis(1, 0, 0);
		GDrawCappedCone3D(0.5 * m_bodyWidth, (i == 0 ? 0.25 : 0.5) * m_bodyWidth, m_bodyLength);
		GDrawPopTransformation();
	    }
	    
	}
    }

    
    void MPSnakeSimulator::DrawSimple(void)
    {
	double        T3[Algebra3D::Trans_NR_ENTRIES];
	double        R3[Algebra3D::Rot_NR_ENTRIES];
	const double *TR;
	GMaterial     gmat;

	MPStandardSimulator::Draw();
		
	if(GDrawIs2D())
	{
	    GDrawColor(1, 0, 0);
	    GDrawQuad2D(&m_bodies[0]);
	    GDrawColor(0, 1, 0);
	    for(int i = 8; i < (int) m_bodies.size(); i += 8)
		GDrawQuad2D(&m_bodies[i]);
	}
	else
	{
	    	    
	    gmat.SetPearl();

	    Algebra3D::IdentityAsRot(R3);
	    T3[2] = 0.0;
	    
	    for(int i = 0; i <= m_nrLinks; ++i)
	    {
		TR = &m_TRs[i * Algebra2D::TransRot_NR_ENTRIES];
		
		T3[0] = TR[0];
		T3[1] = TR[1];
		
		R3[0] = R3[4] = TR[2];
		R3[3] = TR[3];
		R3[1] = -R3[3];
	
		if(i == 0)
		    gmat.SetDiffuse(0.0, 0.0, 0.0);
		else
		    gmat.SetDiffuse(0.0, 1, 0.0);
		GDrawMaterial(&gmat);
		GDrawPushTransformation();
		GDrawMultTransRot(T3, R3);
		GDrawMultTrans(0, 0, 0.5 * m_bodyWidth);
		GDrawMultFromZAxisToAxis(1, 0, 0);
		GDrawCappedCone3D(0.5 * m_bodyWidth, (i == 0 ? 0.25 : 0.5) * m_bodyWidth, m_bodyLength);
		GDrawPopTransformation();
	    }
	    
	}
    }
}



