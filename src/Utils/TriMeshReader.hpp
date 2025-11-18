/*
 * Copyright (C) 2023 Erion Plaku
 * All Rights Reserved
 * 
 *   Created by Erion Plaku
 *
 * Code should not be distributed or used without written permission
 * from the copyright holder.
 */


#ifndef GP__TRIMESH_READER_HPP_
#define GP__TRIMESH_READER_HPP_

#include "Utils/TriMesh.hpp"
#include <cstdio>

namespace GP
{
    bool TriMeshReader(const char fname[], TriMesh * const tmesh);
    bool OFFTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool PLYTriMeshReader(const char fname[], TriMesh * const tmesh);    
    bool OBJTriMeshReader(const char fname[], TriMesh * const tmesh);    
    bool STLAsciiTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool STLBinaryTriMeshReader(const char fname[], TriMesh * const tmesh);
    bool StandardTriMeshReader(FILE *out, TriMesh * const tmesh);


    bool STLAsciiTriMeshWriter(const char fname[], TriMesh * const tmesh);
    bool STLBinaryTriMeshWriter(const char fname[], TriMesh * const tmesh);
    bool StandardTriMeshWriter(FILE *in, TriMesh * const tmesh);

}

#endif





    
    

