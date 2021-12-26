.. _compiling_pcl_docker:

Compiling PCL from source on Docker
------------------------------------

This tutorial explains how to build the Point Cloud Library **from source** on
Docker.

.. note::
The walkthrough of the procedure was done in Windows 10 where Ubuntu 20.04 is installed
in virtual machine. The reason is that docker can be much easier installed in linux OSs
compared to Windows

Requirements
-------------
Git installation

Open a terminal in Ubuntu (inside VirtualBox) and run the following command in order to
check if docker is already installed
.. note::
	git --version
If it is not already installed, run again in terminal the following commands
.. note::
	sudo apt install git

Docker installation.

Open a terminal in Ubuntu (inside VirtualBox) and run the following command in order to
check if docker is already installed
.. note::
	docker --version
If it is not already installed, run again in terminal the following commands
.. note::
	sudo apt install docker.io
	sudo snap install docker
Verify if docker is really installed by running docker --version. The containeres
sudo docker ps shows running container. Other useful command to see the docker are
.. note::
	service docker status which shows the docker status. If everything is fine it 
	active (running). You can start/stop docker if needed by running service docker start/stop

Downloading PCL source code
---------------------------
Download the pcl source code in Ubuntu (inside VirtualBox) with
  git clone https://github.com/PointCloudLibrary/pcl.git



Docker container configuration
------------------------------
Pull the docker image by running
.. note::
	sudo docker pull pointcloudlibrary/env:20.04
	
The docker image above will have OS Ubuntu 20.04. 
Other possible images options can be found under
https://github.com/PointCloudLibrary/pcl/blob/master/.ci/azure-pipelines/azure-pipelines.yaml#L17

Do not worry if it takes enough time because all the pcl dependencies will be installed in this 
step. In other word no need to install Boost or Eigen.

Running the container with
.. note::
	sudo docker run -v ~/Desktop/pcl:/home --rm -it pointcloudlibrary/env:20.04 bash
where ~/Desktop/pcl represents the pcl source code in Ubuntu (inside VirtualBox) while 
home represents the pcl source code inside the docker container. Instead of home any other 
path can be used for example home/pcl.  Using volumes,  actions perfomred like creation of
new file in  Ubuntu (inside VirtualBox) are directly mapped to the selected path location
inside docker container. 


Building PCL
--------------
After running the container, we need to nagivate to pcl source code by simply running
cd home. Next step is to create a build folder and to that directory. This can be done
with mkdir build && cd build inside docker container. 

In case you prefer to use a specifi compiler like clang instead of gcc run
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++g

Last step is the cmake configuration which is done by running inside the build folder
.. note::
	cmake ..
Other cmake variables can be passed in this step for example cmake -DCMAKE_BUILD_TYPE=Release ..
which will change the build target to “Release”

Finally compile everything by running
.. note::
	make -j2

Installing PCL
--------------
Install the result:
make -j2 install 
or
sudo make -j2 install



