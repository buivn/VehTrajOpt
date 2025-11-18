/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "Utils/Misc.hpp"
#include "Utils/PrintMsg.hpp"
#include <algorithm>
#include <fstream>

namespace GP
{
    void PointAtDistanceAlongPath(const int n, const double pts[], const double d, const int dim, double p[])
    {
	double dtot = 0.0;
	double dcurr= 0.0;
	
	for(int i = 1; i < n; ++i)
	{
	    dcurr = Algebra::PointDist(dim, &pts[dim * (i-1)], &pts[dim * i]);
	    dtot += dcurr;
	    if(dtot > d)
	    {
		const double *p1 = &pts[dim * (i-1)];
		const double *p2 = &pts[dim * i];
		const double  t = (d - dtot + dcurr) / dcurr;
		for(int j = 0; j < dim; ++j)
		    p[j] = (1 - t) * p1[j] + t * p2[j];
		return;
	    }
	}
	for(int j = 0; j < dim; ++j)
	    p[j] = pts[dim * (n - 1) + j];
    }

    
    void RegularizePointsAlongPath(const int n, 
				   const double pts[], 
				   const double dsep, 
				   const int    dim,
				   std::vector<double> *new_pts)
    {
	double dtot = 0.0;
	double dcurr= 0.0;
	double d    = dsep;
	

	for(int j = 0; j < dim; ++j)
	    new_pts->push_back(pts[j]);
	
	for(int i = 1; i < n; ++i)
	{
	    dcurr = Algebra::PointDist(dim, &pts[dim * (i-1)], &pts[dim * i]);
	    dtot += dcurr;
	    while(dtot > d)
	    {
		
		const double *p1 = &pts[dim * (i-1)];
		const double *p2 = &pts[dim * i];
		const double  t = (d - dtot + dcurr) / dcurr;
		for(int j = 0; j < dim; ++j)
		    new_pts->push_back((1 - t) * p1[j] + t * p2[j]);

		//printf("dtot = %f d = %f dcurr = %f t = %f\n", dtot, d, dcurr, t);
		
		d += dsep;
	    }
	}
	for(int j = 0; j < dim; ++j)
	    new_pts->push_back(pts[dim * (n - 1) + j]);
    }

    
    void GenerateGapsU(const int nrGaps,
		       const double gapDurations[],
		       const double duration,		      
		       const double minNonGap,
		       const int nrMustUseTimes,
		       const double mustUseTimes[],
		       std::vector<double> * const gaps)
    {
	gaps->clear();
	
	std::vector<double> times;
	bool acceptable;
	bool hasGapAtStart = false;
	bool hasGapAtEnd = false;
	
	times.resize(nrGaps+1);
	times[nrGaps] = duration;
	do
	{
	    for(int i = nrMustUseTimes - 1; i >= 0; --i)
		times[i] = mustUseTimes[i];
	    for(int i = nrMustUseTimes; i < nrGaps; ++i)
		times[i] = RandomUniformReal(minNonGap, duration - minNonGap);
	    std::sort(times.begin(), times.end());
	    if(times[0] < Constants::EPSILON)
	    {
		hasGapAtStart = true;
		times[0] = 0.0;
	    }	    
	    if(times[nrGaps - 1] + Constants::EPSILON >= duration)
	    {
		hasGapAtEnd = true;
		times[nrGaps - 1] = duration - gapDurations[nrGaps - 1];
		acceptable = nrGaps <= 1 || (times[nrGaps - 1] > times[nrGaps - 2]);
	    }
	    else
		acceptable = true;
	    for(int i = 0; i < (nrGaps - 1 + (hasGapAtEnd == false)) && acceptable; ++i)
		acceptable = times[i] + gapDurations[i] + minNonGap < times[i + 1];
	    if(hasGapAtStart == false)
		acceptable = acceptable && times[0] >= minNonGap;
	    

	}
	while(acceptable == false);

	for(int i = 0; i < nrGaps; ++i)
	{
	    gaps->push_back(times[i]);
	    gaps->push_back(times[i] + gapDurations[i]);
	}
	//for(int i = 0; i < gaps->size(); ++i)
	//    printf("%f ", (*gaps)[i]);
	//printf("...ugaps\n");
	
    }
    
    
    void GenerateGaps(const int nrGaps,
		      const double duration,
		      const double minGap,
		      const double maxGap,
		      const double minNonGap,
		      std::vector<double> * const gaps)
    {
	double start = 0.0;

	gaps->clear();
	
	for(int n = nrGaps; n > 0; --n)
	{
	    double remaining = duration - start -  n * minNonGap - maxGap * (n - 1);
	    double g1Time    = RandomUniformReal(minGap, maxGap);

	    if(minNonGap >= remaining - g1Time)
	    {
		OnInputError(printf("check your arguments...smth is wrong\n"));
		return;
		
	    }
	    double g1Start   = RandomUniformReal(minNonGap, remaining - g1Time);
	    gaps->push_back(start + g1Start);
	    gaps->push_back(start + g1Start + g1Time);
	    start   += g1Start + g1Time;
	}
    
    }

    int NrLinesFile(const char fname[])
    {
	std::ifstream in(fname);

	if(!in)
	    return 0;
	
	std::string line;

	int nrLines = 0;	
	while(!in.eof())
	{
	    std::getline(in, line);
	    ++nrLines;
	}
	if(nrLines > 0)
	    return nrLines - 1;
	return 0;
    }

    
    bool NextCounter(const int n, int vals[], const int max[])
    {
	int carry = 1;
	for(int i = n - 1; i >= 0; --i)
	{
	    vals[i] += carry;
	    carry = vals[i] >= max[i];
	    vals[i] = vals[i] % max[i];
	}
	
	return carry == 0;
    }
    
}	



