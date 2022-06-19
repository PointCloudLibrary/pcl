FROM pointcloudlibrary/env:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
      doxygen-latex \
      dvipng \
      graphviz \
      git \
      python3-pip \
      pandoc \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install Jinja2 sphinx sphinxcontrib-doxylink sphinx_rtd_theme requests grip
