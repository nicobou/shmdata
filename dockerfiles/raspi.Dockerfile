ARG SHMDATA_IMAGE="registry.gitlab.com/sat-metalab/shmdata"
ARG SHMDATA_TAG="develop"

FROM ${SHMDATA_IMAGE}:${SHMDATA_TAG} AS dependencies
LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

# Install common dependencies
RUN apt-get update -y \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get install -y --no-install-recommends -qq \
            build-essential \
            ca-certificates \
            cmake bison \
            flex \
            gcc-8 \
            g++-8 \
            git \
            gsoap \
            gstreamer1.0-libav \
            gstreamer1.0-plugins-bad \
            gstreamer1.0-plugins-base \
            gstreamer1.0-plugins-good \
            gstreamer1.0-plugins-ugly \
            jackd \
            libcgsi-gsoap1\
            libcgsi-gsoap-dev \
            libcurl3-gnutls \
            libcurl4-gnutls-dev \
            libxxf86vm1 \
            libgl1-mesa-dev \
            libglib2.0 \
            libglib2.0-dev \
            libgstreamer-plugins-base1.0 \
            libgstreamer-plugins-base1.0-dev \
            libgstreamer-plugins-bad1.0-dev \
            libgstreamer1.0 \
            libgstreamer1.0-dev \
            libjack-jackd2-dev \
            libjson-glib-1.0-0 \
            libjson-glib-dev \
            liblo7 \
            liblo-dev \
            libltc11 \
            libltc-dev  \
            libportmidi0 \
            libportmidi-dev \
            libpulse0 \
            libpulse-mainloop-glib0 \
            libpulse-dev \
            libsamplerate0 \
            libsamplerate0-dev \
            libsoup2.4-dev \
            libssl1.1 \
            libssl-dev \
            libtool \
            libvncclient1 \
            libvncserver1 \
            libvncserver-dev \
            libxcursor1 \
            libxcursor-dev \
            libxinerama1 \
            libxinerama-dev \
            libxrandr2 \
            libxrandr-dev \
            linux-libc-dev \
            mesa-utils \
            python3 \
            libpython3.7-stdlib \
            libpython3.7 \
            python3-dev \
            libc6-dev \
            uuid \
            uuid-dev \
            wah-plugins \
            xorg

FROM dependencies AS build
LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

ARG BUILD_DIR="/opt/switcher"
ARG BUILD_TYPE="Release"
ARG ENABLE_GPL="ON"

# Set all CMakes environment variables
ARG C_COMPILER="gcc-8"
ARG CXX_COMPILER="g++-8"

# Set switcher paths
WORKDIR ${BUILD_DIR}
COPY . ${BUILD_DIR}

RUN mkdir build \
    && cd build \
    && CC="${C_COMPILER}" \
        CXX="${CXX_COMPILER}" \
        cmake \
            -DENABLE_GPL="${ENABLE_GPL}" \
            -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" .. \
    && make -j1 \
    && make install \
    && ldconfig

FROM build AS clean
LABEL MAINTAINER="Metalab <metalab-dev@sat.qc.ca>"

ARG BUILD_DIR="/opt/switcher"

# Remove build folder and clean apt cache
RUN rm -rf ${BUILD_DIR} \
    && apt remove -y -qq \
        build-essential \
        cmake \
        bison \
        flex \
        git \
        libcgsi-gsoap-dev \
        libcurl4-gnutls-dev \
        libgl1-mesa-dev \
        libglib2.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        libgstreamer1.0-dev \
        libjack-jackd2-dev \
        libjson-glib-dev \
        liblo-dev \
        libltc-dev  \
        libportmidi-dev \
        libpulse-dev \
        libsamplerate0-dev \
        libssl-dev \
        libsoup2.4-dev \
        libtool \
        libvncserver-dev \
        libxcursor-dev \
        libxinerama-dev \
        libxrandr-dev \
        linux-libc-dev \
        python3-dev \
        uuid-dev \
    && apt autoclean \
    && apt autoremove -y \
    && rm -rf /var/lib/{apt,dpkg,cache,log}/ 

