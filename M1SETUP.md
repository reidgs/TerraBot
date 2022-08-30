# Installing `TerraBot` on Apple Silicon (M1)

## 1. Obtain `Conda` for managing Python installations
- Do *not* use the Anaconda installer, instead prefer [`miniconda`](https://docs.conda.io/en/latest/miniconda.html) or [`miniforge / mambaforge`](https://github.com/conda-forge/miniforge)
    - Recommended to use [miniconda](https://docs.conda.io/en/latest/miniconda.html) 
        ```bash
        # in Downloads/
        wget https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh
        chmod a+x Miniconda3-latest-MacOSX-arm64.sh
        ./Miniconda3-latest-MacOSX-arm64.sh
        ```

## 2. Install `ros-noetic` to a specialized conda env

(full instructions obtained from [here](https://github.com/RoboStack/ros-noetic))

```bash
# if you don't have mamba yet, install it first in the base environment (not needed when using mambaforge):
conda install mamba -c conda-forge

mamba create -n 15-482 ros-noetic-desktop python=3.9 -c robostack -c robostack-experimental -c conda-forge --no-channel-priority --override-channels

# activate the new conda environment
mamba init
source ~/.zshrc # or .bashrc (basically restart the terminal)
conda activate 15-482

# optionally, install some compiler packages if you want to e.g. build packages in a catkin_ws:
mamba install cmake pkg-config make ninja

# reload environment to activate required scripts before running anything
conda deactivate && conda activate 15-482

# if you want to use rosdep, also do:
mamba install rosdep
rosdep init  # note: do not use sudo!
rosdep update

# install 15-482 TerraBot dependencies
pip install panda3d install transitions sklearn ortools opencv-python matplotlib
```

## 3. Clone `TerraBot` and open it:
```bash
conda activate 15-482 # (do all your 15-482 work in this env)
git clone https://github.com/reidgs/TerraBot
cd TerraBot
python TerraBot.py -m sim -s 1 -g
```