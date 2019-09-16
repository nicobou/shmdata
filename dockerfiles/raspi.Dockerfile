FROM ubuntu

LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

# set shmdata paths
WORKDIR /tmp/shmdata
COPY . /tmp/shmdata

RUN apt update -y \
    # install common dependencies
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends -qq \
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
        ca-certificates \
    # build and install shmdata library
    && mkdir -p build \
    && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
      # remove build folder and clean apt cache
    && rm -rf /tmp/shmdata \
    && apt remove -y -qq \
        cmake \
        bison \
        build-essential \
        flex \
        libtool \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        python3-dev \
    && apt-get autoclean \
    && apt-get autoremove -y \
    && rm -rf /var/lib/{apt,dpkg,cache,log}/
