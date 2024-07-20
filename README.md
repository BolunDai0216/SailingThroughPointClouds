# Sailing Through Point Clouds

This repo contains the official implementation of [**Sailing Through Point Clouds: Safe Navigation Using Point Cloud Based Control Barrier Functions**](https://arxiv.org/pdf/2403.18206).

## Usage

First clone the repo 

```bash
git clone https://github.com/BolunDai0216/SailingThroughPointClouds.git
```

The only supported way to run the code is via a devcontainer on a linux machine. To build the devcontainer, first run the setup script in inside `provisioning` to create the `.zsh_history` file which will be mounted to the devcontainer

```bash
cd /path/to/provisioning && bash setup.sh
```

Then, open a VS Code window at the root of this repo

```bash
cd /path/to/SailingThroughPointClouds && code . 
```

Then press `Shift + Ctrl + P` and select `Dev Containers: Rebuild and Reopen in Container` which builds the devcontainer. After the container is built, first generate the height map data by running the dataset generation script in `scripts`

```bash
mkdir data && cd /path/to/scripts && python3 dataset_gen.py
```

This will save the generated height maps inside the newly created `data` folder. Then, to run the simulation run the command

```bash
cd /path/to/scripts && python3 sim.py
```

## Citation

To cite our paper, please use the following bibtex

```bibtex
@article{DaiKKGTK23,
  author       = {Bolun Dai and Rooholla Khorrambakht and Prashanth Krishnamurthy and Farshad Khorrami},
  title        = {Sailing Through Point Clouds: Safe Navigation Using Point Cloud Based Control Barrier Functions},
  journal      = {{IEEE} Robotics and Automation Letters},
  year         = {2024},
}
```
