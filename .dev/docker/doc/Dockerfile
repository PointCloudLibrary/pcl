FROM pointcloudlibrary/env:20.04

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

# Use eps2eps to solve https://github.com/doxygen/doxygen/issues/7484 before Doxygen 1.8.18
RUN update-alternatives --install /usr/local/bin/ps2epsi ps2epsi /usr/bin/ps2epsi 1 \
 && update-alternatives --install /usr/local/bin/ps2epsi ps2epsi /usr/bin/eps2eps 1000
