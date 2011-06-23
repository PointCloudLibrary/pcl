/*************************************************************
  Generalized-ICP Copyright (c) 2009 Aleksandr Segal.
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
*************************************************************/



#include "scan.h"
#include <stdio.h>


int dgc_scan_t::save(const char* filename) {
  int i;
  int n = points.size();
  int m = norms.size();
    
  FILE *f = fopen(filename, "w");
  if(f == NULL) {
    fprintf(stderr, "Couldn't open file.\n");
    return 0;
  }
  fwrite(&n, sizeof(int),1, f);
  fwrite(&pose, sizeof(dgc_pose_t),1, f);
  
  for(i = 0; i < n; i++) {
    fwrite(&points[i], sizeof(dgc_vector3d_c_t), 1, f);
  }
  fwrite(&m, sizeof(int),1, f);
  for(i = 0; i < m; i++) {
    fwrite(&norms[i], sizeof(dgc_vector3d_c_t), 1, f);
  }

  fclose(f);
  
  return 1;
}


int dgc_scan_t::load(const char *filename) {
  int i, n, m;
  points.clear();
  norms.clear();

  FILE *f = fopen(filename, "r");
  if(f == NULL) {
    fprintf(stderr, "Couldn't open file.\n");
    return 0;
  }
  fread(&n, sizeof(int), 1, f);
  fread(&pose, sizeof(dgc_pose_t), 1, f);
  
  dgc_vector3d_c_t pt;
  for(i = 0; i < n; i++) {
    fread(&pt, sizeof(dgc_vector3d_c_t), 1, f);
    points.push_back(pt);
  }
  
  fread(&m, sizeof(int), 1, f);
  dgc_vector3d_c_t nu;
  for(i = 0; i < m; i++) {
    fread(&nu, sizeof(dgc_vector3d_c_t), 1, f);
    norms.push_back(nu);
  }
  fclose(f);
  return 1;
}
