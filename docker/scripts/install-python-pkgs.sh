#!/bin/bash

set -e

# Install the Go2Py in editable mode
cd /home && git clone https://github.com/Rooholla-KhorramBakht/Go2Py.git && cd Go2Py && python3 -m pip install -e .

# Install torch
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install other Python packages
pip3 install \
    black \
    casadi \
    hydra-core \
    ipykernel \
    isort \
    matplotlib \
    mujoco \
    opencv-python \
    pin \
    proxsuite \
    scikit-learn \
    scipy \
    warp-lang