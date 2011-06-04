cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("PCL Tutorials")

foreach(subdir 
          pcd_read 
          pcd_write
          concatenate_fields
          concatenate_points
          openni_grabber
          template_alignment
       )

  add_subdirectory(${subdir})

endforeach()
