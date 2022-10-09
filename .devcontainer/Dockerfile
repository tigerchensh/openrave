# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.245.2/containers/debian/.devcontainer/base.Dockerfile

# [Choice] Debian version (use bullseye on local arm64/Apple Silicon): bullseye, buster
ARG VARIANT="buster"
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}

# ** [Optional] Uncomment this section to install additional packages. **
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>

RUN apt -y update && \
    apt install -y --no-install-recommends build-essential cmake doxygen \
    g++ git octave python-dev python-setuptools wget python2 curl minizip gettext \
    libxml2-dev libassimp-dev libboost-dev libboost-python-dev libeigen3-dev \
    libbullet-dev libode-dev liblapack-dev libfcl-dev coreutils nano libminizip-dev libpcre3-dev \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools liblog4cxx-dev liboctave-dev libmpfr-dev \
    libgmp-dev libboost-all-dev libavformat-dev libavcodec-dev libswscale-dev \
    libxvidcore-dev libbz2-dev libsoqt520-dev libcollada-dom2.4-dp-dev octomap-tools

RUN git config --global user.email "docker@example.com"
RUN git config --global user.name "docker"
RUN git clone https://github.com/pybind/pybind11.git  && \
    cd pybind11 && git checkout v2.2.4 && mkdir build && cd build && \
    git remote add woody https://github.com/woodychow/pybind11.git && \
    git fetch woody && git checkout v2.2.4 && \
    git cherry-pick 94824d68a037d99253b92a5b260bb04907c42355 && \
    git cherry-pick 98c9f77e5481af4cbc7eb092e1866151461e3508 && \
    cmake .. -DPYBIND11_TEST=OFF -DPythonLibsNew_FIND_VERSION=2  && \
    make install

RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && python get-pip.py
RUN pip install ipython h5py numpy sympy pyopengl msgpack

# && git checkout 7dddd054628e42ab973bdbd1f9ab94535beb4d03
RUN git clone https://github.com/Tencent/rapidjson.git && \
    cd rapidjson && mkdir build && \
    cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j `nproc` && make install 



