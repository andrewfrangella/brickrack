# Build environment for TrimUI Brick / Brick Pro (tg5040, aarch64).
# Debian bullseye (glibc 2.31, GCC 10) produces binaries that run on the
# stock TrimUI/NextUI userland, which ships SDL2 in /usr/lib and /usr/trimui/lib.
FROM --platform=linux/arm64 debian:bullseye-slim
# bullseye is end-of-life: packages now live on archive.debian.org.
RUN printf 'deb http://archive.debian.org/debian bullseye main\ndeb http://archive.debian.org/debian bullseye-updates main\ndeb http://archive.debian.org/debian-security bullseye-security main\n' > /etc/apt/sources.list \
    && rm -f /etc/apt/sources.list.d/* \
    && apt-get -o Acquire::Check-Valid-Until=false update \
    && apt-get install -y --no-install-recommends \
      g++ make cmake pkg-config libsdl2-dev ca-certificates file \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
