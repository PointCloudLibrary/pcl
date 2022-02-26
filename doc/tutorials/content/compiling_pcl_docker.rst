.. _compiling_pcl_docker:

Compiling PCL from source on Docker
===================================

This tutorial explains how to build the Point Cloud Library **from source** on docker.

.. note::

   The walkthrough of the procedure was done in Windows 10 host machine where Ubuntu 20.04
   was installed in virtual machine. The reason is that docker can be much easier installed
   in linux OSs compared to Windows

Requirements
-------------
Open a terminal in Ubuntu (inside VirtualBox) and run the coresponding commands from each 
installation section

* Curl installation 

  Check if curl is already installed::

  $ curl --version

  If it is not already installed, run in terminal the relative command for your OS::
  `<https://www.tecmint.com/install-curl-in-linux>`_

* Git installation

  Check if git is already installed::

  $ git --version

  If it is not already installed, run in terminal the relative command for your OS::
  `<https://git-scm.com/download/linux>`_ 

* Docker installation

  Check if docker is already installed::

  $ docker --version

  If it is not already installed, follow the instructions from 
  `<https://github.com/docker/docker-install>`_ and run in terminal::

  $ curl -fsSL https://get.docker.com -o get-docker.sh
  $ sh get-docker.sh

  Other useful commands are()::

  $ docker ps 
  $ service docker status

  The first one shows the running containers while the latter shows the docker status. 
  If everything is fine it will be active (running).
  You can start/stop docker if needed by running::

  $ service docker start/stop


.. note::

   It might be required to add sudo in docker commands if permissions are not set properly.
   See part **run docker commands without sudo** on how to set them correctly so the sudo command is not required.

Downloading  PCL source code
----------------------------
Download the pcl source code in Ubuntu (inside VirtualBox)::

  $ git clone https://github.com/PointCloudLibrary/pcl.git

Docker container configuration
------------------------------
* To run docker commands without sudo::

  $ sudo groupadd docker
  $ sudo usermod -aG docker $USER
  $ newgrp docker

  Verify you can run docker without sudo::

  $ docker run hello-world

* Pull the docker image by running::

  $ docker pull pointcloudlibrary/env:20.04

  Do not worry if it takes a long time because all the required pcl dependencies are preinstalled.
  In other words, there is no need to install Boost, Eigen or other dependencies manually.

  The docker image above will have OS Ubuntu 20.04. Other possible available images can be found under::

  $ curl -L -s 'https://registry.hub.docker.com/v2/repositories/pointcloudlibrary/env/tags/' | jq '.results | map({(."name"): .last_updated}) | add'

  jq can be installed with a single command depending on your OS . See `<https://stedolan.github.io/jq/download/>`_ for more details

.. note::

   It is also possible to use the Dockerfile under .dev folder to set up your docker 
   image. The method of pulling the official docker image is considered more 
   stable option though.

* Run the container::

  $ docker run --user $(id -u):$(id -g) -v $PWD/pcl:/home --rm -it pointcloudlibrary/env:20.04 bash

  where $PWD:/pcl:/home represents the pcl source code in Ubuntu (inside VirtualBox) while
  home represents the pcl source code inside the docker container.
 
  Using volumes, actions performed on a file in Ubuntu (inside VirtualBox) such as creating new files are directly mapped
  to the selected path location inside docker container.

  To exit the container simply run in terminal exit

Building PCL
--------------
After running the container, we need to navigate to pcl source code and create a build folder in that directory.

  $ cd home && mkdir build && cd build

In case you prefer to use a specific compiler like clang instead of gcc run::

  $ export CC=/usr/bin/clang
  $ export CXX=/usr/bin/clang++

Last step is the cmake configuration which is done by running this inside the build folder::

  $ cmake ..

Other cmake variables can be passed in this step for example cmake -DCMAKE_BUILD_TYPE=Release ..
which will change the build target to “Release”. More details about cmake variables can be found
in :ref:`building_pcl`.

Finally compile everything by running::

  $ make -j2

Installing PCL
--------------
Install the result on docker::

  $ make -j2 install

To get root access for just install command::

  $ docker exec -it <container_name>

Next steps
----------
All the steps mentioned in this tutorial should be performed at least once and
after that just running the container command and building or installing is
enough. Periodically though it is needed to pull the latest image to have
possible updates that are incorporated in the meantime.

