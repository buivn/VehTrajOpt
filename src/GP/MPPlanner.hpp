/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MP_PLANNER_HPP_
#define GP__MP_PLANNER_HPP_

#include "GP/Solution.hpp"
#include "GP/Decomposition.hpp"
#include "GP/MPSimulator.hpp"
#include "Utils/Params.hpp"
#include "Utils/Flags.hpp"
#include "Utils/Misc.hpp"

namespace GP
{
    class MPPlanner
    {
    public:
	MPPlanner(void) :
	    m_decomp(NULL),
	    m_sim(NULL)
	{
	}
	
	virtual ~MPPlanner(void)
	{
	}
	
	virtual Decomposition* GetDecomposition(void)
	{
	    return m_decomp;
	}
	
	virtual MPSimulator* GetSimulator(void)
	{
	    return m_sim;
	}
	
	virtual void SetDecomposition(Decomposition *decomp)
	{
	    m_decomp = decomp;
	}
	
	virtual void SetSimulator(MPSimulator *sim)
	{
	    m_sim = sim;
	}
	
	
	virtual void SetupFromParams(Params * const p)
	{
	}
	
	virtual void Run(const double tmax) = 0;
 	virtual int  GetSolved(void) const = 0;
	virtual int GetBest(void) const = 0;	
	virtual void GetReversePath(const int vid, std::vector<int> * const rpath) const = 0;
	virtual void GetPath(const int vid, std::vector<int> * const path) const
	{
	    GetReversePath(vid, path);
	    /*for(auto & val : *path)
		printf("%d ", val);
	    printf(": %d [original]\n", path->size());
	    */
	    ReverseItems<int>(path);
/*
	      for(auto & val : *path)
		printf("%d ", val);
	    printf(": %d [reversed]\n", path->size());
*/	  
	}

	virtual void GetSolution(Solution * const sol) const
	{
	    auto vidSolved = GetSolved();
	    auto vidBest = GetBest();
	    printf("vidSolved = %d vidBest = %d nrStates = %d\n", vidSolved, vidBest, GetNrStates());
	    
	    if(GetSolved() >= 0)
		return GetSolution(GetSolved(), sol);
	    return GetSolution(GetBest(), sol);
	    
	}
	
	virtual void GetSolution(const int vid,  Solution * const sol) const;
		
	virtual int           GetNrStates(void) const = 0;
	virtual const double* GetState(const int i) const = 0;
	virtual void          Draw(void) const = 0;
	
	virtual void ResetForNewMP(void)
	{
	}
	
    protected:	
	Decomposition *m_decomp;
	MPSimulator   *m_sim;
    };     
}

#endif

