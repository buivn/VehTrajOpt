/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__ID_GENERATOR_HPP_
#define GP__ID_GENERATOR_HPP_


namespace GP
{
    class IdGenerator
    {
    public:
	IdGenerator(void) : m_id(0) 
	{
	}
	
	virtual ~IdGenerator(void)
	{
	}
	
	virtual void Reset(const int id = 0)
	{
	    m_id = id;
	}
	
	virtual int New(void)
	{
	    return m_id++;
	    
	}
	
	
	static IdGenerator* GetSingleton(void)
	{
	    return m_singleton;
	}
	
    protected:
	static IdGenerator *m_singleton;
	int m_id;
	
    };
    
}

#endif



