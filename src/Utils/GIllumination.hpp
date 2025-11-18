/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__GILLUMINATION_HPP_
#define GP__GILLUMINATION_HPP_

#include "Utils/GLight.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace GP
{
    class GIllumination
    {
    public:
	GIllumination(void)
	{
	    AddDefaultLights();
	}
	
	
	
	virtual ~GIllumination(void)
	{
	    RemoveLights();
	}
	
	enum Property
	    {
		AMBIENT = 0,
		DIFFUSE   
	    };

	virtual void SetupFromParams(Params * const p);
		
	virtual const float* GetProperty(const Property type) const
	{
	    return m_prop[type];
	}
	
	virtual void SetProperty(const Property type, const float * const prop)
	{
	    SetProperty(type, prop[0], prop[1], prop[2], prop[3]);
	}
	
	virtual void SetProperty(const Property type,
				 const float a = 0.0f,
				 const float b = 0.0f,
				 const float c = 0.0f,
				 const float d = 1.0f)
	{
	    m_prop[type][0] = a;
	    m_prop[type][1] = b;
	    m_prop[type][2] = c;
	    m_prop[type][3] = d;
	}
	
	void SetAmbient(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(AMBIENT, a, b, c, d);
	}
	void SetDiffuse(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(DIFFUSE, a, b, c, d);
	}

	void CopyFrom(const GIllumination * const gIllum)
	{
	    SetProperty(AMBIENT,   gIllum->GetProperty(AMBIENT));
	    SetProperty(DIFFUSE,   gIllum->GetProperty(DIFFUSE));

	    DeleteItems<GLight*>(&m_lights);
	    for(int i = 0; i < gIllum->GetNrLights(); ++i)
	    {
		m_lights[i] = new GLight();
		m_lights[i]->CopyFrom(gIllum->GetLight(i));
	    }
	}

	int GetNrLights(void) const
	{
	    return m_lights.size();
	}
	
	GLight* GetLight(const int i)
	{
	    return m_lights[i];
	}
	
	const GLight* GetLight(const int i) const
	{
	    return m_lights[i];
	}

	virtual void AddLight(GLight * const light)
	{
	    m_lights.push_back(light);
	}

	virtual void AddDefaultLights(void)
	{
	    GLight *light;
	    const float h = 50.0f;
	    	    
	      light = new GLight();
	    light->SetPosition(40.0, 40.0, h);
	    AddLight(light);

	    light = new GLight();
	    light->SetPosition(-40.0, 40.0, h);
	    AddLight(light);
	    
	    light = new GLight();
	    light->SetPosition(40.0, -40.0, h);
	    AddLight(light);

	     light = new GLight();
	    light->SetPosition(-40.0, -40.0, h);
	    AddLight(light);
	    

	      light = new GLight();
	    light->SetPosition(0.0, 0.0, h);
	     AddLight(light);
	}
	
	
	virtual void RemoveLights(void)
	{
	    DeleteItems<GLight*>(&m_lights);
	    m_lights.clear();
	}
	
	
    protected:
	float                m_prop[2][4];
	std::vector<GLight*> m_lights;
    };
}


#endif
    
    






