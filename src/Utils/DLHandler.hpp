/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__DL_HANDLER_HPP_
#define GP__DL_HANDLER_HPP_

namespace GP
{
    namespace DLHandler
    {
	void* GetSymbol(void *handle, const char * const name);
	void* GetSymbol(const char * const name);
    }
}

#endif
