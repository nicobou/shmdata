# FROM ubuntu:disco 
FROM nvidia/cudagl:10.1-base-ubuntu18.04
MAINTAINER Metalab <metalab-dev@sat.qc.ca>
# Docker Repository
ARG BRANCH
RUN apt-get update -y \
        && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -qq cmake bison build-essential flex libtool git libgstreamer1.0 libgstreamer1.0-dev libgstreamer-plugins-base1.0 libgstreamer-plugins-base1.0-dev python3 python3-dev \
        && git clone --depth=1 --branch=${BRANCH} https://gitlab.com/sat-metalab/shmdata.git \
	&& cd shmdata \
	&& mkdir build \
	&& cd build \
	&& cmake .. \
	&& make -j"$(nproc)" \
	&& make install \
	&& ldconfig \
	&& cd .. \
	&& rm -rf shmdata \
	&& apt remove -y -qq cmake bison build-essential flex libtool git libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev  python3-dev \
	&& apt-get autoclean && apt-get autoremove -y && rm -rf /var/lib/{apt,dpkg,cache,log}/ \
