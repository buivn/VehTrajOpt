/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "Utils/Stats.hpp"
namespace GP
{
    
    Stats* Stats::m_singleton = new Stats();

    double Stats::GetValue(const char id[]) 
    {
	auto iter = m_values.find(id);
	if(iter != m_values.end())
	    return iter->second;
	return -1.0;
    }

    
    double Stats::AddValue(const char id[], const double t)
    {
	auto iter = m_values.find(id);
	if(iter != m_values.end())
	{
	    iter->second += t;
	    return iter->second;
	}
	else
	{
	    m_values.insert(std::make_pair(id, t));
	    return t;
	}
    }
    
    
    void Stats::PrintValues(FILE *out) const
    {
	for(auto iter = m_values.begin(); iter != m_values.end(); iter++)
	    fprintf(out, "%-50s %8.6f\n", iter->first.c_str(), iter->second);
    }
    
}



