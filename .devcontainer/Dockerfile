#FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04
#
#ENV DEBIAN_FRONTEND=noninteractive
#
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    cmake \
#    ninja-build \
#    gcc-arm-none-eabi \
#    libnewlib-arm-none-eabi \
#    git \
#    build-essential \
#    gdb-multiarch \
#    libgtest-dev \
#    software-properties-common \
#    clang-format \
#    clang-tidy \
#    && add-apt-repository -y ppa:ppa-verse/cling \
#    && apt-get update \
#    && apt-get install -y --no-install-recommends cling \
#    && apt-get clean \
#    && rm -rf /var/lib/apt/lists/*
#
#WORKDIR /workspaces
FROM mcr.microsoft.com/devcontainers/cpp:ubuntu-24.04

RUN apt-get update && apt-get install -y \
    wget \
    git \
    cmake \
    ninja-build \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    git \
    gdb-multiarch \
    libgtest-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Miniforge
RUN wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
    -O /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p /opt/conda \
    && rm /tmp/miniforge.sh

ENV PATH="/opt/conda/bin:${PATH}"

# Install Cling
RUN conda install -y -c conda-forge cling \
    && conda clean -afy

WORKDIR /workspaces