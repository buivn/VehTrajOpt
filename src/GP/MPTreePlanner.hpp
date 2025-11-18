/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MP_TREE_PLANNER_HPP_
#define GP__MP_TREE_PLANNER_HPP_

#include "GP/MPPlanner.hpp"
#include "Utils/Map.hpp"

namespace GP
{
    class MPTreePlanner : public MPPlanner
    {
    public:
	enum Status
	    {
		EXTEND_NORMAL    = 0,
		EXTEND_COLLISION = 1,
		EXTEND_TARGET    = 2,
		EXTEND_GOAL      = 3
	    };
	    
	MPTreePlanner(void);
	
	virtual ~MPTreePlanner(void);

	virtual void SetupFromParams(Params * const p);
	
	virtual int GetSolved(void) const
	{
	    return m_vidSolved;
	}

	virtual int GetBest(void) const
	{
	    return m_vidBest;
	}
	

	virtual void GetReversePath(const int vid, std::vector<int> * const rpath) const;

	virtual int GetNrStates(void) const
	{
	    return m_vertices.size();
	}
	
	virtual const double* GetState(const int i) const
	{
	    return m_vertices[i]->m_state;
	}
	
	virtual void Draw(void) const;

	double  m_dtolSteer;
	double  m_dtolSample;
	int     m_extendMaxNrSteps;
	int     m_extendMinNrSteps;
	double  m_probSteer;
	double  m_goalBias;

	
   protected:
	struct Vertex
	{
	    Vertex(void)
	    {
		m_parent = Constants::ID_UNDEFINED;
		m_state  = NULL;
		m_rid    = Constants::ID_UNDEFINED;
		m_nextWaypt    = Constants::ID_UNDEFINED;
		m_tpred = -1.0;//RandomUniformReal(0.4, 0.8);		
	    }
	    
	    virtual ~Vertex(void)
	    {
		if(m_state)
		    delete[] m_state;
	    }
	   
	    int     m_parent;
	    double *m_state;
	    int     m_rid;
	    int     m_nextWaypt;
	    double m_tpred;
	};

	virtual int GetNrVertices(void) const
	{
	    return m_vertices.size();
	}
	
	virtual Vertex* NewVertex(void) const
	{
	    return new Vertex();
	}
		 
	virtual void Initialize(void);

	virtual Status ExtendFrom(const int    vid, 
				  const double target[]);

	virtual int AddVertex(Vertex * const v);

	std::vector<Vertex*> m_vertices;
	int                  m_vidSolved;
	int m_vidBest;
	double m_costToGoalBest;
	
    public:
	
	virtual void ResetForNewMP(void)
	{
	    MPPlanner::ResetForNewMP();

	    m_costToGoalBest = INFINITY;	    
	    m_vidSolved = m_vidBest = Constants::ID_UNDEFINED;
	    DeleteItems<Vertex*>(&m_vertices);
	    m_vertices.clear();
	    
	}
	

    };     
}

#endif

