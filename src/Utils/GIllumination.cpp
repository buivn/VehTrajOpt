/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#include "Utils/GIllumination.hpp"

namespace GP
{
    void GIllumination::SetupFromParams(Params * const p)
    {
	const float ambx  = (float) p->GetValueAsDouble("GraphicsIlluminationAmbientX", 0.2);
	const float amby  = (float) p->GetValueAsDouble("GraphicsIlluminationAmbientY", 0.2);
	const float ambz  = (float) p->GetValueAsDouble("GraphicsIlluminationAmbientZ", 0.2);
	const float diffx = (float) p->GetValueAsDouble("GraphicsIlluminationDiffuseX", 1.0);
	const float diffy = (float) p->GetValueAsDouble("GraphicsIlluminationDiffuseY", 1.0);
	const float diffz = (float) p->GetValueAsDouble("GraphicsIlluminationDiffuseZ", 1.0);

	SetProperty(AMBIENT, ambx, amby, ambz);
	SetProperty(DIFFUSE, diffx, diffy, diffz);

	const int n = p->GetValueAsInt("GraphicsNrLights", 0);
	char      msg[200];
	
	if(n == 0)
	    return;

	RemoveLights();
	
	
	for(int i = 0; i < n; ++i)
	{
	    sprintf(msg, "GraphicsLight%dPosX", i);
	    const float posx = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dPosY", i);
	    const float posy = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dPosZ", i);
	    const float posz = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dAmbientX", i);
	    
	    const float ambx = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dAmbientY", i);
	    const float amby = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dAmbientZ", i);
	    const float ambz = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    
	    sprintf(msg, "GraphicsLight%dDiffuseX", i);
	    const float diffx = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dDiffuseY", i);
	    const float diffy = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dDiffuseZ", i);
	    const float diffz = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    
	    sprintf(msg, "GraphicsLight%dSpecularX", i);
	    const float specx = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dSpecularY", i);
	    const float specy = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    sprintf(msg, "GraphicsLight%dSpecularZ", i);
	    const float specz = (float) p->GetValueAsDouble(msg, HUGE_VAL);
	    
	    GLight *light = new GLight();
	    
	    if(posx != HUGE_VAL && posy != HUGE_VAL && posz != HUGE_VAL)
		light->SetPosition(posx, posy, posz);
	    if(ambx != HUGE_VAL && amby != HUGE_VAL && ambz != HUGE_VAL)
		light->SetAmbient(ambx, amby, ambz);
	    if(diffx != HUGE_VAL && diffy != HUGE_VAL && diffz != HUGE_VAL)
		light->SetDiffuse(diffx, diffy, diffz);
	    if(specx != HUGE_VAL && specy != HUGE_VAL && specz != HUGE_VAL)
		light->SetSpecular(specx, specy, specz);
	    
	    AddLight(light);
	    
	}
    }	
}

    
    






