# AReS: Adaptive Reachable Set Learning

## Prerequisite

- **Python version**: 3.9 or above
- **Conda**: Make sure `conda` is installed on your system. You can download it from [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Anaconda](https://www.anaconda.com/products/distribution).

## Installation Guide

### Step 1: Clone the Repository

Clone this repository to your local machine:
```bash
git clone https://github.com/your-repo/AReS.git
cd AReS
```

### Step 2: Create a Conda Environment
Create and activate a new Conda environment:
```bash
conda create -n ares_env python=3.9
conda activate ares_env
```

### Step 3: Install Required Libraries
Install all required libraries using the requirements.txt file:
```bash
pip install -r requirements.txt
```

## AReS Simulation
### Step 1: Run the Simulation
Execute the simulation code with the desired experiment name
```sh
python srcs/sim_mav.py --exp_name [exp_name]
```

### Step 2: Visualize the Results
Generate and save plots for the simulation results
```sh
python plots/plot_next_state_pred.py --dafa_fn "output/[exp_name]/data.pk" --baseline_data_fn "output/baseline/data.pk" --fig_root "plots/[foleder_name]"
```
