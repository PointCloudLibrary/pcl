find_package (VTK REQUIRED)
execute_process (COMMAND echo ${VTK_INCLUDE_DIRS} COMMAND sed "s/ / -I/g")

