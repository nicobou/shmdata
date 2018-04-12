    FROM ubuntu:bionic
    MAINTAINER Metalab <metalab-dev@sat.qc.ca>
    # Docker Repository
    ARG shmdata_version
    ARG current_branch
    COPY ci/install_shmdata.sh /opt/
    RUN apt-get update -y \
        && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq git cmake bison build-essential flex libtool libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libjson-glib-dev libcgsi-gsoap-dev \
        gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly liblo-dev linux-libc-dev libpulse-dev libportmidi-dev libjack-jackd2-dev gsoap wah-plugins \
        jackd libvncserver-dev uuid-dev libssl-dev libgl1-mesa-dev swh-plugins libltc-dev  libcurl4-gnutls-dev libxrandr-dev libxinerama-dev libxcursor-dev libsamplerate0-dev \
	    && apt-get clean && apt-get autoclean && apt-get autoremove -y && rm -rf /var/lib/{apt,dpkg,cache,log}/ \
        && /opt/install_shmdata.sh ${current_branch} ${shmdata_version}
