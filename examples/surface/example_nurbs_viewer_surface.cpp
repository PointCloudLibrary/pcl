#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ Point;

int
main (int argc, char *argv[])
{
  std::string file_3dm;

  if (argc < 2)
  {
    printf ("\nUsage: pcl_example_nurbs_viewer_surface 3dm-out-file\n\n");
    exit (0);
  }
  file_3dm = argv[1];

  pcl::visualization::PCLVisualizer viewer ("B-spline surface viewer");
  viewer.setSize (800, 600);

  int mesh_resolution = 128;

  ON::Begin();

  // load surface
  ONX_Model on_model;
  bool rc = on_model.Read(file_3dm.c_str());

  // print diagnostic
  if ( rc )
    std::cout << "Successfully read: " << file_3dm << std::endl;
  else
    std::cout << "Errors during reading: " << file_3dm << std::endl;

//  ON_TextLog out;
//  on_model.Dump(out);

  if(on_model.m_object_table.Count()==0)
  {
    std::cout << "3dm file does not contain any objects: " << file_3dm << std::endl;
    return -1;
  }

  const ON_Object* on_object = on_model.m_object_table[0].m_object;
  if(on_object==NULL)
  {
    std::cout << "object[0] not valid." << std::endl;
    return -1;
  }

  const ON_NurbsSurface& on_surf = *(ON_NurbsSurface*)on_object;

  pcl::PolygonMesh mesh;
  std::string mesh_id = "mesh_nurbs";
  if(on_model.m_object_table.Count()==1)
  {
    std::cout << "3dm file does not contain a trimming curve: " << file_3dm << std::endl;


    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (on_surf, mesh, mesh_resolution);
  }
  else
  {
    on_object = on_model.m_object_table[1].m_object;
    if(on_object==NULL)
    {
      std::cout << "object[1] not valid." << std::endl;
      return -1;
    }

    const ON_NurbsCurve& on_curv = *(ON_NurbsCurve*)on_object;

    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (on_surf, on_curv, mesh,
                                                                     mesh_resolution);
  }

  viewer.addPolygonMesh (mesh, mesh_id);

  viewer.spin ();
  return 0;
}

