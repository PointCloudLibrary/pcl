Requirements:
-------------
- install pcl 1.7 from it's official repository
- install python3 from official repository
- download, compile and install swig 2.0.11 from http:/otes:/www.swig.org/
- install the latest jdk
- download and install netbeans 7.4 , to run and use testingApp (test the interfaces)
- install latest cmake , don't forget to make  sure that cmake successfully found jni liberaries and pcl libraries
- linux operating system

Compiling instructions:
-----------------------
- go to the installation directory (PCL_Wrapper) using cd command
- run the compilation script ./exec.sh

notes:
------
- during  don't panic if you see alot of warnings from swig , redefinition warning are ok, others may not
- currently if you want to use a template class type, you must instaniate it in swig files, for 
	example:
		if you want to use "pcl::PointCloud<pcl::PointXYZRGB>" then you must use that "%template(PointCloud_PointXYZRGB) pcl::PointCloud<pcl::PointXYZRGB>;" (you will find me doing that in the  file so do as you see)
		and by making that and running ./exec.sh , you will find wrapper.pcl.PointCloud_PointXYZRGB class which is you desired one
- currently i tested and interfaced the classes that was used in pcl first tutorial (How 3D Features work in PCL)
