/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__MAP_HPP_
#define GP__MAP_HPP_

#include "Utils/HashFn.hpp"
#include <unordered_map>

#define UseMap(Key, Data)  std::unordered_map<Key, Data, HashStruct<Key> >


#endif
    
    
    
    







