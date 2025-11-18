/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "Utils/Params.hpp"

namespace GP
{
    
    Params* Params::m_singleton = new Params();

    Params::~Params(void)
    {
	
	for(auto & iter : m_values)
	    if(iter.second)
		free(iter.second);
	m_values.clear();
	
    }
    
    const char* Params::GetValue(const char id[], const char notFound[]) 
    {
	auto iter = m_values.find(id);
	if(iter != m_values.end())
	    return iter->second;
	return notFound;
    }
    
    void Params::SetValue(const char id[], const char val[])
    {
	m_values[id] =  strdup(val);
    }
    
    
    void Params::Print(FILE *out) const
    {
	for(auto &iter : m_values)
	    fprintf(out, "<%-30s> <%s>\n", iter.first.c_str(), iter.second);
    }

    void Params::ParseArgs(const int start,  const int end, char **argv)
    {
	for(int i = start; (i+1) <= end; i += 2)
	{
	    if(strcmp(argv[i], "ParamsExtraFile") == 0)
	    {
		printf("...using ParamsExtraFile <%s>\n", argv[i+1]);
		
		FILE *fp = fopen(argv[i+1], "r");
		Read(fp);
		fclose(fp);
	    }
	    else
		SetValue(argv[i], argv[i + 1]);
	}
	
    }

    void Params::Read(FILE *in)
    {
	char id[300];
	char val[300];
	while(fscanf(in, "%s %s", id, val) == 2)
	{
	    // printf("reading <%s> <%s>\n", id, val);
	    if(strcmp(id, "ParamsExtraFile") == 0)
	    {
		printf("...using ParamsExtraFile <%s>\n", val);
		
		FILE *fp = fopen(val, "r");
		Read(fp);
		fclose(fp);
	    }
	    else
		SetValue(id, val);
	}
    }
}



