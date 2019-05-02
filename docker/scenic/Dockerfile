ARG SWITCHER_BRANCH
FROM switcher:${SWITCHER_BRANCH}
COPY run_scenic.sh /
MAINTAINER Metalab <metalab-dev@sat.qc.ca>
# Docker Repository
ARG BRANCH
RUN apt-get update -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -qq \
       build-essential \
       pkg-config \
       curl \
       libjson-glib-1.0-0 \
       libjson-glib-dev \
       libgstreamer1.0 \
       libgstreamer1.0-dev \
       h2o \
       git \
       libicu-dev \
    && curl -sL https://deb.nodesource.com/setup_8.x | bash - \
    && apt install -y nodejs \
    && cd \
    && git clone --depth=1 --branch=${BRANCH} https://gitlab.com/sat-mtl/telepresence/scenic-core.git \
    && cd scenic-core \
    && make build \
    && make install \
    && cd \
    && git clone --depth=1 --branch=${BRANCH} https://gitlab.com/sat-mtl/telepresence/scenic.git \
    && cd scenic \
    && echo "SERVER_PORT=8000" > .env \
    && make build \
    && make install \
    && apt remove -y -qq \
       build-essential \
       pkg-config \
       git \
    && apt autoclean && apt autoremove -y && rm -rf /var/lib/{apt,dpkg,cache,log}/ 

CMD ["bash", "/run_scenic.sh"]
