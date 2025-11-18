/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__GLIGHT_HPP_
#define GP__GLIGHT_HPP_

namespace GP
{
    class GLight
    {
    public:
	GLight(void)
	{
	    SetProperty(AMBIENT, 0.2f, 0.2f, 0.2f);
	    SetProperty(DIFFUSE, 1.0f, 1.0f, 1.0f);
	    SetProperty(SPECULAR, 0.5f, 0.5f, 0.5f);
	    SetProperty(POSITION, 0.0f, 0.0f, 0.0f);
	}
	
	virtual ~GLight(void)
	{
	}
	
	enum Property
	    {
		AMBIENT = 0,
		DIFFUSE,
		SPECULAR,
		POSITION   
	    };
	
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
	void SetSpecular(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(SPECULAR, a, b, c, d);
	}
	void SetPosition(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetProperty(POSITION, a, b, c, d);
	}
	
	void CopyFrom(const GLight * const gLight)
	{
	    SetProperty(AMBIENT,   gLight->GetProperty(AMBIENT));
	    SetProperty(DIFFUSE,   gLight->GetProperty(DIFFUSE));
	    SetProperty(SPECULAR,  gLight->GetProperty(SPECULAR));
	    SetProperty(POSITION,  gLight->GetProperty(POSITION));
	}

    protected:
	float m_prop[4][4];
    };
}


#endif
    
    






