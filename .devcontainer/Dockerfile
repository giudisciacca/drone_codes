FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04

RUN apt-get update && apt-get install -y \
    cmake \
    ninja-build \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    git \
    build-essential \
    gdb-multiarch \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces