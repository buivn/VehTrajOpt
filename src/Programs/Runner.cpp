/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */
#include "Utils/DLHandler.hpp"
#include "Utils/PseudoRandom.hpp"
#include <cstdio>
 #include <dlfcn.h>

int main(int argc, char **argv)
{
    typedef int (*MainFcn) (int, char **);

    GP::RandomSeed();
   
    if(argc < 2)
	printf("usage: Runner <program_name> [list_of_program_args]\n");
    else
    {
	MainFcn fcn = (MainFcn) GP::DLHandler::GetSymbol(argv[1]);
	if(fcn)
	{
	    printf("Executing program <%s> with %d args\n", argv[1], argc - 1);
	    for(int i = 1; i < argc; ++i)
		printf("  <%s>\n", argv[i]);
	    
	    return fcn(argc - 1, &(argv[1]));
	}
	else
	    printf("Program <%s> not found %p\n", argv[1], dlsym(RTLD_DEFAULT, argv[1]));
    }    

    return 0;
}
