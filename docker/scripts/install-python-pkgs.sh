#!/bin/bash

set -e

# Install the Go2Py in editable mode
cd /home && git clone https://github.com/Rooholla-KhorramBakht/Go2Py.git && cd Go2Py && python3 -m pip install -e .

# Install Python dependencies
pip3 install scipy ipykernel warp-lang
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
pip3 install matplotlib opencv-python proxsuite
pip3 install isort black
pip3 install warp-lang scikit-learn casadi mujoco pin