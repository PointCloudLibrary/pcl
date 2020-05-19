FROM pointcloudlibrary/env:20.04

# This will make apt-get install without question
ARG DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
       flex \
       bison \
       git \
       llvm-dev \
       clang \
       libclang-dev \
       cm-super-minimal \
       ghostscript \
       graphviz \
       dvipng \
       texlive-latex-recommended \
       texlive-latex-extra \
       texlive-fonts-recommended \
       texlive-font-utils \
       texlive-extra-utils \
       python3-pip \
       python3-setuptools \
       python3-wheel \
       pandoc \
 && rm -rf /var/lib/apt/lists/*

# Build doxygen with clang-parsing capabilities (with the patch from https://github.com/doxygen/doxygen/issues/6379#issuecomment-547465097)
RUN git clone -b Release_1_8_15 --single-branch https://github.com/doxygen/doxygen.git \
 && sed -i '264d' doxygen/src/CMakeLists.txt \
 && mkdir doxygen/build \
 && cd doxygen/build \
 && cmake -DCMAKE_BUILD_TYPE=Release -Duse_libclang=YES ../ \
 && cmake --build . --parallel ${nproc} --target install \
 && cd ../.. \
 && rm -rf doxygen

# Install Sphinx and related dependencies
RUN pip3 install Jinja2 sphinx sphinxcontrib-doxylink sphinx_rtd_theme requests grip
