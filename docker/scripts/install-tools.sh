#!/bin/bash

set -e

# Install tools, utilities, and etc.

apt-get update
apt-get install -y --no-install-recommends \
    curl \
    git \
    git-lfs \
    iputils-ping \
    net-tools \
    ninja-build \
    psmisc \
    python3-pip \
    software-properties-common \
    unzip \
    usbutils \
    vim \
    wget \
    zsh

pip3 install \
    cmakelang==0.6.13