# ⛵ $\textsf{\color{darkorange}{Sailing\ Through\ Point\ Clouds}}$ 🌤️

[![License](https://img.shields.io/badge/License-MIT-cfd8dc?style=flat-square&labelColor=orange&color=lightgray)](https://github.com/BolunDai0216/SailingThroughPointClouds/blob/main/LICENSE)

This repo contains the official implementation of [**Sailing Through Point Clouds: Safe Navigation Using Point Cloud Based Control Barrier Functions**](https://arxiv.org/pdf/2403.18206).

## ⌨️ $\textsf{\large\color{Dandelion}{Usage}}$

First, clone the repo and pull the meshes using the following commands

```bash
git clone https://github.com/BolunDai0216/SailingThroughPointClouds.git

# Git LFS
git lfs pull
```

> [!NOTE]
> The only supported way to run the code is via a dev container on a Linux machine. **However (maybe with some changes), the code will work on other platforms.**

To build the dev container, first run the setup script in inside `provisioning` to create the `.zsh_history` file, which will be mounted to the dev container

```bash
cd /path/to/provisioning && bash setup.sh
```

Then, open a VS Code window at the root of this repo

```bash
cd /path/to/SailingThroughPointClouds && code . 
```

Then press `Shift + Ctrl + P` and select `Dev Containers: Rebuild and Reopen in Container`, which builds the dev container. After the container is built, first generate the height map data by running the dataset generation script in `scripts`

```bash
mkdir data && cd /path/to/scripts && python3 dataset_gen.py
```

This will save the generated height maps inside the newly created `data` folder. Then, to run the simulation, run the command

```bash
cd /path/to/scripts && python3 sim.py
```

## 📖 $\textsf{\large\color{Dandelion}{Citation}}$

To cite our paper, please use the following BibTeX

```bibtex
@article{DaiKKK24,
  author       = {Bolun Dai and Rooholla Khorrambakht and Prashanth Krishnamurthy and Farshad Khorrami},
  title        = {Sailing Through Point Clouds: Safe Navigation Using Point Cloud Based Control Barrier Functions},
  journal      = {{IEEE} Robotics and Automation Letters},
  year         = {2024},
  volume       = {9},
  number       = {9},
  pages        = {7731-7738},
}
```
