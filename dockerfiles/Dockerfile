FROM nvidia/cudagl:10.1-base-ubuntu18.04 AS dependencies
LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

# Note: the option --no-install-recommends fails unit tests 

# Install common dependencies
RUN apt update -y \
   && DEBIAN_FRONTEND=noninteractive apt install -y -qq \
      cmake \
      bison \
      build-essential \
      flex \
      libtool \
      libgstreamer1.0 \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0 \
      libgstreamer-plugins-base1.0-dev \
      python3 \
      python3-dev \
   # clean apt cache
   && apt-get clean \
   && apt-get autoclean \
   && apt-get autoremove -y \
   && rm -rf /var/lib/{apt,dpkg,cache,log}/

FROM dependencies AS build
LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

ARG BUILD_DIR="/opt/shmdata"
ARG BUILD_TYPE="Release"

# Set shmdata paths
WORKDIR ${BUILD_DIR}
COPY . ${BUILD_DIR}

# Build and install shmdata library
RUN mkdir -p build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig

FROM build AS clean
LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

ARG BUILD_DIR="/opt/shmdata"
    
# remove build folder and clean apt cache
RUN rm -rf ${BUILD_DIR} \
    && apt remove -y -qq \
        cmake \
        bison \
        build-essential \
        flex \
        libtool \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        python3-dev \
    && apt autoclean \
    && apt autoremove -y \
    && rm -rf /var/lib/{apt,dpkg,cache,log}/