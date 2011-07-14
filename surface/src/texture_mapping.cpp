/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: texture_mapping.cpp 1006 2011-07-13 13:07:00 ktran $
 *
 */

/** \author Khai Tran */
#include <pcl/surface/texture_mapping.h>

///////////////////////////////////////////////////////////////////////////////////////////////
pcl::TextureMapping::TextureMapping(){}
///////////////////////////////////////////////////////////////////////////////////////////////
pcl::TextureMapping::~TextureMapping(){}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setF(float f){
  this->f = f;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setVectorField(float x, float y, float z){
  this->vector_field =  Eigen::Vector3f(x, y, z);
  // normalize vector field
   this->vector_field = this->vector_field/std::sqrt(this->vector_field.dot(this->vector_field));
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setTextureFiles( std::vector<char*> tex_files){
  this->tex_files = tex_files;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setTextureMaterials(TexMaterial tex_material){
  this->tex_material = tex_material;
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::TextureMapping::setTextureMesh(TextureMesh tex_mesh){
  this->tex_mesh = tex_mesh;
}
///////////////////////////////////////////////////////////////////////////////////////////////
int pcl::TextureMapping::blendTexture2Faces(std::vector<pcl::PointXY> pp1, std::vector<pcl::PointXY> pp2, IplImage* img_tex1, IplImage* img_tex2){

  // resample each line to blending
  float delta = 0;
  int x1, y1, x2, y2;
  float dis = std::sqrt((pp1[0].x - pp1[1].x) * (pp1[0].x - pp1[1].x) + (pp1[0].y - pp1[1].y) * (pp1[0].y - pp1[1].y));
  if (dis ==0) return 1;
  while(delta < dis){
    delta += 0.5;
    x1 = floor(pp1[0].x + (pp1[1].x - pp1[0].x) *  delta/dis);
    y1 = floor(pp1[0].y + (pp1[1].y - pp1[0].y) *  delta/dis);

    x2 = floor(pp2[0].x + (pp2[1].x - pp2[0].x) *  delta/dis);
    y2 = floor(pp2[0].y + (pp2[1].y - pp2[0].y) *  delta/dis);
    int rgb;
    for (int i=-2; i< 3; i++)
      for(int j=-2; j< 3; j++){
        for(int k=0; k <img_tex1->nChannels; k++){ // blending
          if(x1+j >=0 && x1+j < img_tex1->width && x2+j >=0  && x2+j < img_tex2->width &&
          y1+i >=0 && y1+i < img_tex1->height && y2+i >=0 && y2+i < img_tex2->height)
          {
            rgb = img_tex1->imageData[(y1+i)* img_tex1->widthStep+(x1+j)*img_tex1->nChannels+k];
            img_tex1->imageData[(y1+i)* img_tex1->widthStep+(x1+j)*img_tex1->nChannels+k] =
            img_tex2->imageData[(y2+i)* img_tex2->widthStep+(x2+j)*img_tex2->nChannels+k];
            img_tex2->imageData[(y2+i)* img_tex2->widthStep+(x2+j)*img_tex2->nChannels+k] = rgb;
          }
        }
      }
  }
  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////
float pcl::TextureMapping::getDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  return std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}
///////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::PointXY> pcl::TextureMapping::mapTexture2Face(pcl::PointXYZ pp1, pcl::PointXYZ pp2, pcl::PointXYZ pp3){
  std::vector<pcl::PointXY> tex_coordinates;
  // process for each face
  Eigen::Vector3f p1p2(pp2.x - pp1.x, pp2.y - pp1.y, pp2.z - pp1.z);
  Eigen::Vector3f p1p3(pp3.x - pp1.x, pp3.y - pp1.y, pp3.z - pp1.z);
  Eigen::Vector3f p2p3(pp3.x - pp2.x, pp3.y - pp2.y, pp3.z - pp2.z);

  // Normalize
  p1p2 = p1p2/std::sqrt(p1p2.dot(p1p2));
  p1p3 = p1p3/std::sqrt(p1p3.dot(p1p3));
  p2p3 = p2p3/std::sqrt(p2p3.dot(p2p3));

  // compute vector normal of a face
  Eigen::Vector3f f_normal = p1p2.cross(p1p3);
  f_normal = f_normal/std::sqrt(f_normal.dot(f_normal));
  // project vector field onto the face:
  // vector v1_projected = v1 - Dot(v1, n) * n;

  Eigen::Vector3f f_vector_field =  vector_field - vector_field.dot(f_normal) * f_normal;
  // Normalize
  f_vector_field = f_vector_field/std::sqrt(f_vector_field.dot(f_vector_field));
  PointXY tp1, tp2, tp3;

  double alpha = std::acos(f_vector_field.dot(p1p2));

  // distance between 3 vertices of triangles
  double e1 = getDistance(pp2, pp3)/f;
  double e2 = getDistance(pp1, pp3)/f;
  double e3 = getDistance(pp1, pp2)/f;

  // initialize
  tp1.x = 0.0;
  tp1.y = 0.0;

  tp2.x = e3;
  tp2.y = 0.0;

  // determine texture coordinate tp3;

  double cos_p1 = (e2*e2+e3*e3-e1*e1)/(2*e2*e3);
  double sin_p1 = sqrt(1-(cos_p1*cos_p1));

  tp3.x = cos_p1*e2;
  tp3.y = sin_p1*e2;

  // rotating by alpha (angle between V and pp1 & pp2)
  PointXY r_tp2, r_tp3;
  r_tp2.x = tp2.x*std::cos(alpha) - tp2.y*std::sin(alpha);
  r_tp2.y = tp2.x*std::sin(alpha) + tp2.y*std::cos(alpha);

  r_tp3.x = tp3.x*std::cos(alpha) - tp3.y*std::sin(alpha);
  r_tp3.y = tp3.x*std::sin(alpha) + tp3.y*std::cos(alpha);

  // shifting

  tp1.x = tp1.x;
  tp2.x = r_tp2.x;
  tp3.x = r_tp3.x;
  tp1.y = tp1.y;
  tp2.y = r_tp2.y;
  tp3.y = r_tp3.y;

  float min_x = tp1.x;
  float min_y = tp1.y;
  if (min_x > tp2.x) min_x = tp2.x;
  if (min_x > tp3.x) min_x = tp3.x;
  if (min_y > tp2.y) min_y = tp2.y;
  if (min_y > tp3.y) min_y = tp3.y;

  if(min_x < 0){
          tp1.x = tp1.x -min_x;
          tp2.x = tp2.x -min_x;
          tp3.x = tp3.x -min_x;
  }
  if(min_y < 0){
          tp1.y = tp1.y -min_y;
          tp2.y = tp2.y -min_y;
          tp3.y = tp3.y -min_y;
  }

  tex_coordinates.push_back(tp1);
  tex_coordinates.push_back(tp2);
  tex_coordinates.push_back(tp3);
  return tex_coordinates;
}
///////////////////////////////////////////////////////////////////////////////////////////////
pcl::TextureMesh pcl::TextureMapping::mapTexture2Mesh(){
  // mesh information
  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  int point_size = tex_mesh.cloud.data.size () / nr_points;


  // temporary PointXYZ
  float x_, y_, z_;
  // temporary face
  PointXYZ facet[3];
  // triangle verties
  int t_vertices[3];

  // Texture images list
  std::vector< std::vector<IplImage*> >img_T;

  // texture list
  std::vector<IplImage*> T;
  // texture coordinates for each mesh
  std::vector< std::vector<PointXY> > texture_map;

  // face index
  size_t nr_faces =0;
  // store 2 connected faces
  Eigen::MatrixXd edge_cache(nr_points, nr_points);

  // initialization
  for (int i=0; i < nr_points; ++i)
          for (int j=0; j< nr_points; ++j) edge_cache(i,j) = 0;

  for (size_t m = 0; m < tex_mesh.tex_polygons.size(); ++m){
          // list of texon for each mesh
    std::vector<IplImage*> img_T_tmp;
    // texture coordinates for each mesh
    std::vector<PointXY> texture_map_tmp;

    // loading texture image for mesh m
    IplImage* img_tex_base = cvLoadImage(tex_files[m], 1);

    // create texture sample =  10 times bigger image
    IplImage* img_tex = cvCreateImage(cvSize(img_tex_base->width*10, img_tex_base->height * 10), 8, 3);
    for (int t_h = 0; t_h < 10; ++t_h ){
      for (int ii=0; ii < img_tex_base->height; ++ii)
        for (int t_w = 0; t_w < 10; ++t_w )
          for (int jj=0; jj < img_tex_base->width; ++jj)
            for(int kk=0; kk< img_tex_base->nChannels; ++kk)
              img_tex->imageData[(ii+t_h*img_tex_base->width)*img_tex->widthStep+(jj+t_w*img_tex_base->height)*img_tex->nChannels+ kk] =
              img_tex_base->imageData[ii*img_tex_base->widthStep+jj*img_tex_base->nChannels+kk];
    }

    // processing for each face
    // face index
    if (m > 0) nr_faces += tex_mesh.tex_polygons[m-1].size();
    for (size_t i=0; i < tex_mesh.tex_polygons[m].size(); ++i){
      int idx;
      for (size_t j=0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j){
        idx =  tex_mesh.tex_polygons[m][i].vertices[j];
        t_vertices[j] = idx;
        memcpy (&x_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[0].offset], sizeof (float));
        memcpy (&y_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[1].offset], sizeof (float));
        memcpy (&z_, &tex_mesh.cloud.data[idx * point_size + tex_mesh.cloud.fields[2].offset], sizeof (float));
        facet[j].x = x_;
        facet[j].y = y_;
        facet[j].z = z_;
      }
      if(edge_cache(t_vertices[0], t_vertices[1]) == 0){
        edge_cache(t_vertices[0], t_vertices[1]) = i+nr_faces;
      }else{
        if(edge_cache(t_vertices[1], t_vertices[0]) == 0){
          edge_cache(t_vertices[1], t_vertices[0]) = i+nr_faces;
        }
      }
      if(edge_cache(t_vertices[0], t_vertices[2]) == 0){
        edge_cache(t_vertices[0], t_vertices[2]) = i+nr_faces;
      }else{
        if(edge_cache(t_vertices[2], t_vertices[0]) == 0){
          edge_cache(t_vertices[2], t_vertices[0]) = i+nr_faces;
        }
      }
      if(edge_cache(t_vertices[1], t_vertices[2]) == 0){
          edge_cache(t_vertices[1], t_vertices[2]) = i+nr_faces;
      }else{
        if(edge_cache(t_vertices[2], t_vertices[1]) == 0){
          edge_cache(t_vertices[2], t_vertices[1]) = i+nr_faces;
        }
      }

      // boundary of texture
      cv::Rect tex_bound;
      std::vector<PointXY> tex_coordinates = mapTexture2Face(facet[0], facet[1], facet[2]);

      // image processing
      float min_x = tex_coordinates[0].x;
      float min_y = tex_coordinates[0].y;
      float max_x = tex_coordinates[0].x;
      float max_y = tex_coordinates[0].y;
      for(size_t n =1; n < tex_coordinates.size(); ++n){
        if (min_x > tex_coordinates[n].x) min_x = tex_coordinates[n].x;
        if (min_y > tex_coordinates[n].y) min_y = tex_coordinates[n].y;
        if (max_x < tex_coordinates[n].x) max_x = tex_coordinates[n].x;
        if (max_y < tex_coordinates[n].y) max_y = tex_coordinates[n].y;
      }
      tex_bound.x = 0;
      tex_bound.y = 0;
      tex_bound.width  = (max_x - min_x) * img_tex_base->width;
      tex_bound.height = (max_y - min_y) * img_tex_base->height;

      for(size_t n =0; n < tex_coordinates.size(); ++n){
        tex_coordinates[n].x = tex_coordinates[n].x/max_x;
        tex_coordinates[n].y = tex_coordinates[n].y/max_y;
      }
      // get ROI image from sample texture
      cvSetImageROI (img_tex, tex_bound);
      IplImage* img_ROI = cvCreateImage(cvGetSize(img_tex), img_tex->depth, img_tex->nChannels);
      cvCopy(img_tex, img_ROI);
      cvResetImageROI(img_tex);
      img_T_tmp.push_back(img_ROI);
      for(size_t n =0; n < tex_coordinates.size(); ++n) texture_map_tmp.push_back(tex_coordinates[n]);
    }// end faces
    img_T.push_back(img_T_tmp);
    texture_map.push_back(texture_map_tmp);
  }// end meshes
  /* edge blending*/

  int vf1_idx[3];
  int vf2_idx[3];
  for (int i = 0; i < nr_points; ++i){
    for(int j = 0; j< nr_points; ++j){
      if (edge_cache(i,j) !=0 && edge_cache(j,i) !=0){
        // get 2 connected faces & vertices
        size_t f1_idx = edge_cache(i,j);
        size_t f2_idx = edge_cache(j,i);
        // reset cache
        edge_cache(i,j) = 0;
        edge_cache(j,i) = 0;
        // determine the mesh of the face belong to
        int m1 = 0;
        int m2 =0;
        nr_faces = 0;
        size_t f1_idx_local = f1_idx;
        size_t f2_idx_local = f2_idx;
        for(size_t m=0; m <tex_mesh.tex_polygons.size(); ++m){
          nr_faces += tex_mesh.tex_polygons[m].size();
          if(f1_idx >= nr_faces - tex_mesh.tex_polygons[m].size()  && f1_idx < nr_faces ){ // belong to mesh m
            f1_idx_local = f1_idx - (nr_faces - tex_mesh.tex_polygons[m].size());
            for (size_t v = 0; v < tex_mesh.tex_polygons[m][f1_idx - (nr_faces - tex_mesh.tex_polygons[m].size()) ].vertices.size(); ++v){ // get vertices of face 1
              vf1_idx[v] =  tex_mesh.tex_polygons[m][f1_idx - (nr_faces - tex_mesh.tex_polygons[m].size())].vertices[v];
            }
            m1 = m;
          }
          if(f2_idx >= nr_faces - tex_mesh.tex_polygons[m].size()  && f2_idx < nr_faces ){ // belong to mesh m
            f2_idx_local = f2_idx - (nr_faces - tex_mesh.tex_polygons[m].size());
            for (size_t v = 0; v < tex_mesh.tex_polygons[m][f2_idx - (nr_faces - tex_mesh.tex_polygons[m].size())].vertices.size(); ++v){  // get vertices of face 2
              vf2_idx[v] =  tex_mesh.tex_polygons[m][f2_idx - (nr_faces - tex_mesh.tex_polygons[m].size())].vertices[v];
            }
            m2 = m;
          }
        }// end for m
        // determine the connected edge
        std::vector<PointXY> pp1;
        std::vector<PointXY> pp2;
        PointXY tmp_p;
        // determine order
        for(int p=0; p<3; ++p){
          for(int q=0; q<3; ++q){
            if(vf1_idx[p] == vf2_idx[q]){
              tmp_p.x = texture_map[m1][3*f1_idx_local+p].x *  img_T[m1][f1_idx_local]->width;
              tmp_p.y = (1-texture_map[m1][3*f1_idx_local+p].y)*  img_T[m1][f1_idx_local]->height;
              pp1.push_back(tmp_p);
              tmp_p.x = texture_map[m2][3*f2_idx_local+q].x *  img_T[m2][f2_idx_local]->width;
              tmp_p.y = (1-texture_map[m2][3*f2_idx_local+q].y) *  img_T[m2][f2_idx_local]->height;
              pp2.push_back(tmp_p);
            }
          }
        }
        // edge blending
        blendTexture2Faces(pp1, pp2, img_T[m1][f1_idx_local], img_T[m2][f2_idx_local]);
      }// end if
    }// end for j
  }// end for i

  /* reconstruct texture coordinates with atlas textures */
  // texture atlas
  std::vector<IplImage*> atlas; // maximum atlas size: 2048x2048

  // temporary texture coordinate
  PointXY tmp_VT;

  // for every triangles in each mesh
  for (size_t m = 0; m < tex_mesh.tex_polygons.size(); ++m){
    std::vector<TexMaterial> tmp_material;
    std::vector<size_t>         tex_material_idx;
    std::vector<PointXY>     tmp_coordinates;
    tex_material_idx.push_back(0);

    // number of atlas images
    int nr_atlas =0;

    IplImage* img_atlas = cvCreateImage(cvSize(2048, 2048), 8, 3);

    // create white temporary image
    for (int i=0; i < img_atlas->height; ++i)
      for (int j=0; j < img_atlas->width; ++j)
        for(int k=0; k< img_atlas->nChannels; ++k)
          img_atlas->imageData[i*img_atlas->widthStep+j*img_atlas->nChannels+k] = 255;

    // coordinate of each texture
    int id_w  = 0;
    int id_h  = 0;
    int max_h = 0;

    for (size_t t=0; t < tex_mesh.tex_polygons[m].size(); ++t){

      // texture image for triangle t in mesh m
      IplImage* tmp = img_T[m][t];

      // Arrange the images to atlas
      if((id_w + tmp->width <= img_atlas->width && id_h + tmp->height > img_atlas->height) ||
              (id_w + tmp->width > img_atlas->width && id_h + tmp->height > img_atlas->height))
      { // next image
        std::stringstream tex_name;
        std::stringstream tex_file;
        tex_name << "material_"<< m<< nr_atlas;
        tex_name >> tex_material.tex_name;
        tex_file << "tex_"  << m <<nr_atlas << ".jpg";
        tex_file >> tex_material.tex_file;

        // save atlas image
        cvSaveImage(tex_material.tex_file.c_str(), img_atlas);
        // create new texture material
        tmp_material.push_back(tex_material);
        // save face index
        tex_material_idx.push_back(t);
        nr_atlas++;
        // reset to white image
        for (int i=0; i < img_atlas->height; ++i)
                for (int j=0; j < img_atlas->width; ++j)
                        for(int k=0; k< img_atlas->nChannels; ++k)
                                img_atlas->imageData[i*img_atlas->widthStep+j*img_atlas->nChannels+k] = 255;
        // reset indexes
        id_w = 0;
        id_h = 0;
        max_h = 0;
      }
      if(id_w + tmp->width > img_atlas->width && id_h + tmp->height < img_atlas->height){
        id_h += max_h;
        id_w = 0;
        max_h = 0;
      }
      for (int i=0; i < tmp->height; ++i)
        for (int j=0; j < tmp->width; ++j)
          for(int k=0; k< tmp->nChannels; ++k)
            img_atlas->imageData[(i+ id_h)*img_atlas->widthStep+(j+id_w)*img_atlas->nChannels+k]
            = tmp->imageData[i*tmp->widthStep+j*tmp->nChannels+k];
      // relocate texture coordinates
      for (int j=0; j < 3; ++j){
        tmp_VT.x = (texture_map[m][3*t+j].x * tmp->width + id_w)/img_atlas->width;
        tmp_VT.y = (img_atlas->height - id_h - (1-texture_map[m][3*t+j].y) * tmp->height)/img_atlas->height;
        tmp_coordinates.push_back(tmp_VT);
      }
      // debug :: draw triangle on texture
//      cvLine(img_atlas, cvPoint(texture_map[m][3*t+0].x * tmp->width + id_w, id_h + (1-texture_map[m][3*t+0].y) * tmp->height),
//          cvPoint(texture_map[m][3*t+2].x * tmp->width + id_w, id_h + (1-texture_map[m][3*t+2].y) * tmp->height),
//          cvScalar(255, 0, 0, 0), 2, 8, 0);
//      cvLine(img_atlas, cvPoint(texture_map[m][3*t+1].x * tmp->width + id_w, id_h + (1-texture_map[m][3*t+1].y) * tmp->height),
//          cvPoint(texture_map[m][3*t+2].x * tmp->width + id_w, id_h + (1-texture_map[m][3*t+2].y) * tmp->height),
//          cvScalar(0, 255, 0, 0), 2, 8, 0);
//      cvLine(img_atlas, cvPoint(texture_map[m][3*t+0].x * tmp->width + id_w, id_h + (1-texture_map[m][3*t+0].y) * tmp->height),
//          cvPoint(texture_map[m][3*t+1].x * tmp->width + id_w, id_h + (1-texture_map[m][3*t+1].y) * tmp->height),
//          cvScalar(0, 0, 255, 0), 2, 8, 0);
      // end debug
      // increase w index
      id_w += tmp->width;
      // get maximum height of image in the same line
      if (max_h < tmp->height) max_h =tmp->height;
    }

    // the last atlas
    std::stringstream tex_name;
    std::stringstream tex_file;
    tex_name << "material_"<< m << nr_atlas;
    tex_name >> tex_material.tex_name;
    tex_file << "tex_"  << m << nr_atlas << ".jpg";
    tex_file >> tex_material.tex_file;
    tmp_material.push_back(tex_material);
    // save last material
    cvSaveImage(tex_material.tex_file.c_str(), img_atlas);

    // push back material & coordinates
    tex_mesh.tex_materials.push_back(tmp_material);
    tex_mesh.tex_material_idx.push_back(tex_material_idx);
    tex_mesh.tex_coordinates.push_back(tmp_coordinates);
  }
  // return mesh with texture
  return tex_mesh;
}
