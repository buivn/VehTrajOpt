/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__SET_HPP_
#define GP__SET_HPP_

#include "Utils/HashFn.hpp"
#include <unordered_set>

#define UseSet(Key)  std::unordered_set<Key, HashStruct<Key> >

namespace GP
{
    typedef  std::unordered_set< int, HashStruct<int> > IntSet;
}




#endif
    
    
    
    







