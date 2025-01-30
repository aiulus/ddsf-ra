# Behavioral System Theory in Safe Predictive Control

## Overview

This repository implements following algorithms, based on the papers listed under references.
- **Data-Driven Safety Filter (DDSF)** ([1][ddsflink])
- **Data-Enabled Predictive Control (DeePC)** ([2][deepclink])
as well as a custom version of **Model Predictive Control (MPC)** that pairs with the DeePC implementation.

## Repository Structure

```plaintext
.
├── algorithms/
│   ├── ddpc/                # DeePC implementation
│   ├── ddsc/                # DDSF implementation
│   ├── mpc/                 # MPC implementation
├── experiments/             # Experiments on parameter tuning
├── systems/                 # System definitions used by algorithms/mpc and algorithms/deepc
├── utils/                   # Helper functions
│   ├── bst/                 # Functions related to behavioral systems theory
│   ├── data_handling/       # Functions related to data generation and handling
│   ├── evaluation/          # Contains functions for evlauating the performance or behavior of implemented algorithms
│   ├── dynsys/              # Functions related to dynamical systems
│   ├── io/                  # I/O functions
│   ├── linalg/              # Algebraic functions
│   ├── mis/                 # Miscellaneous functions
│   ├── systems/             # Functions that initialize systems
├── docs/                    # Documentation files
└── README.md                # Repository documentation

```

### Prerequisites

- MATLAB (R2022b or newer recommended)
- Optimization Toolbox

### Installation

Clone the repository:

```plaintext
git clone https://github.com/aiulus/ddsf.git
cd ddsf
```
### Usage

The repository offers multiple functions and workflows for data-driven safe predictive control. To illustrate, the experiments/tuneDDSF.m script defines the necessary parameters for running the Data-Driven Safety Filter (DDSF) algorithm and passes them to a wrapper that handles multiple configurations. To run DDSF, simply modify the relevant parameters as needed and follow the steps outlined below.

#### 1. Choose the System Type (`systype`)
The available options are:
- **Linear systems**: `quadrotor`, `damper`, `inverted_pendulum`, `dc_motor`, `cruise_control`, `acc`, `ballNbeam`, `double_pendulum`
- **Nonlinear systems**: `test_nonlinear`, `van_der_pol`, `nonlinear_pendulum`

Example:
```matlab
systype = 'quadrotor';
```
#### 2. Choose the Mode of Execution
Modify in `tuneDDSF.m`:

```matlab
mode = '<chosen_mode>';
```
##### Available Modes:

| Mode    | Description |
|---------|------------|
| `'r'`      | Varies determinant of R (cost matrix). |
| `'nt'`     | Varies both T_ini (initial trajectory length) and N (prediction horizon). |
| `'constr'` | Scales the predefined input/output constraints in `systemsDDSF.m`. |
| `'mixed'`  | Varies all the above simultaneously. |
| `'single'` | Runs DDSF for a single parameter configuration. |

#### 3. Define the Search Space (Optional)

The repository provides predefined search spaces for DDSF experiments, but users can modify `vals` in `tuneDDSF.m` to customize parameter sweeps.

Use default values (no changes needed) or specify a custom search space:

```matlab
vals.quadrotor = struct( ...
    'r', [1, 10, 100, 1000], ... % Custom range for 'r' mode
    'NvsTini', [2 5; 2 10; 2 15], ... % Custom range for 'nt' mode
    'constraints', 1, ... % Custom constraint scaling
    'mixed', struct( ...
        'nt', [2 5; 2 10; 2 15], ...
        'constr', 1, ...
        'R', [1, 10, 100, 1000] ...
    ) ...
);
```

#### 4. Choose Whether to Save Outputs
To save input/output sequences to a CSV file, modify in tuneDDSF.m:

```matlab
toggle_save = 1;
```
The data will be saved in ddsf/outputs/data/.

#### 5. Running the DDSF Algorithm

##### Grid Search Mode (Parameter Exploration)

For parameter tuning (`mode ≠ 'single'`), already implemented in `tuneDDSF.m`:

```matlab
[u, ul, y, yl, descriptions, filename] = ddsfTunerFlex(mode, vals.quadrotor, systype, T_sim, toggle_save);
```
###### Outputs

| Variable      | Description |
|--------------|------------|
| `u`          | Filtered (safe) inputs. |
| `ul`         | Suggested (unsafe) inputs before filtering. |
| `y`          | Filtered (safe) outputs. |
| `yl`         | Outputs projected from `ul`. |
| `descriptions` | Description of the test case (parameters used). |
| `filename`   | Struct with filenames of saved input/output data. |

###### To Visualize Results

```matlab
batchplot(filename.u);
batchplot(filename.y);
```
Plots are saved under ddsf/outputs/plots/.

##### Single Configuration Mode (mode = 'single')
For running DDSF with one fixed configuration, already implemented in tuneDDSF.m:

```matlab
[lookup, time, logs] = runDDSF(systype, T_sim, vals_single);
```

###### Outputs

## Outputs

| Variable  | Description                              |
|-----------|------------------------------------------|
| `lookup`  | DDSF lookup table for system parameters configuration details. |
| `time`    | Simulation time steps.                  |
| `logs`    | Logged input/output sequences.          |

###### To Visualize Results

```matlab
plotDDSF(time, logs, lookup);
```

#### Summary of User Actions

| Step | Action                                      | Where to Change  |
|------|--------------------------------------------|------------------|
| 1.   | Choose a system type (`systype`).         | `tuneDDSF.m`     |
| 2.   | Select a mode (`mode`).                   | `tuneDDSF.m`     |
| 3.   | Customize the search space (`vals`) (optional). | `tuneDDSF.m`     |
| 4.   | Enable or disable saving (`toggle_save`). | `tuneDDSF.m`     |
| 5.   | Run the script and analyze results.       | Already implemented |


## References

1. **Data-Driven Safety Filter (DDSF):**  
   Bajelani, M., & van Heusden, K. (2023). *Data-Driven Safety Filter: An Input-Output Perspective*.  
   [arXiv:2309.00189][ddsflink]

2. **Data-Enabled Predictive Control (DeePC):**  
   Coulson, J., Lygeros, J., & Dörfler, F. (2019). *Data-Enabled Predictive Control: In the Shallows of the DeePC*.  
   [arXiv:1811.05890][deepclink]

   [ddsflink]: https://arxiv.org/abs/2309.00189
   [deepclink]: https://arxiv.org/abs/1811.05890

## Disclaimer 

This project is primarily an academic exercise. While I’ve done my best to faithfully implement the ideas from these papers, please keep in mind:
- **There may be bugs or inaccuracies.** The code hasn’t been extensively validated and might differ in some details from the original papers.
- **Use at your own discretion.**

## Feedback

If you have any feedback or suggestions about this project, please open an [issue](https://github.com/aiulus/ddsf/issues) or contact [aybuke.ulusarslan@tum.de].

## How to Cite This Work

If you wish to use this repository in your work, please cite it as:

```plaintext
@misc{aiulus2025bst,
  author       = {Aybüke Ulusarslan},
  title        = {Behavioral System Theory in Safe Predictive Control},
  year         = {2025},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/aiulus/ddsf}},
}

```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

