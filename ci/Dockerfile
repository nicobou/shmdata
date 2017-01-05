    FROM ubuntu:xenial
    MAINTAINER Metalab <metalab-dev@sat.qc.ca>
    # Docker Repository
    ARG shmdata_version
    ARG current_branch
    COPY install_shmdata.sh /opt/
    RUN apt-get update -y \
        && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq git cmake bison build-essential flex libtool libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libjson-glib-dev libcgsi-gsoap-dev \
        gstreamer1.0-libav gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly liblo-dev linux-libc-dev libgtk-3-dev libpulse-dev libportmidi-dev libjack-jackd2-dev jackd libvncserver-dev uuid-dev libssl-dev libgl1-mesa-dev swh-plugins \
        && apt-get clean && apt-get autoclean && apt-get autoremove -y && rm -rf /var/lib/{apt,dpkg,cache,log}/ \
        && /opt/install_shmdata.sh ${current_branch} ${shmdata_version}
