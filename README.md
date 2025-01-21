# Behavioral Systems Theory in Optimization-Based Control: A Survey

## Overview

This repository implements data-driven, optimization-based control algorithms, featuring:
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
├── experiments/   
├── systems/                 # System definitions used by algorithms/mpc and algorithms/deepc
├── utils/                   # Helper functions
│   ├── bst/                 # Functions related to behavioral systems theory
│   ├── data_handling/       # Functions related to data generation and handling
│   ├── dynsys/              # Functions related to dynamical systems
│   ├── io/                  # I/O functions
│   ├── linalg/              # Algebraic functions
│   ├── mis/                 # Miscellaneous functions
│   ├── systems/             # Functions that initialize systems
├── docs/                    # Documentation files
└── README.md                # Repository documentation

```

## Key Features

- **DDSF: ([1][ddsflink])** Ensures safe operation through system-behavioral filtering of suggested control inputs
- **DeePC: ([2][deepclink])** System-behavioral predictive control
- **Experiments:** Experiments on parameter tuning and performance comparison

## Getting Started

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

Navigate to one of the folders (/algorithms/ddsf, /algorithms/deepc, /algorithms/mpc, /experiments) and follow the instructions in the respective README sections.

## References

1. **Data-Driven Safety Filter (DDSF):**  
   Bajelani, M., & van Heusden, K. (2023). *Data-Driven Safety Filter: An Input-Output Perspective*.  
   [arXiv:2309.00189][ddsflink]

2. **Data-Enabled Predictive Control (DeePC):**  
   Coulson, J., Lygeros, J., & Dörfler, F. (2019). *Data-Enabled Predictive Control: In the Shallows of the DeePC*.  
   [arXiv:1811.05890][deepclink]

[ddsflink]: https://arxiv.org/abs/2309.00189
[deepclink]: https://arxiv.org/abs/1811.05890

## Feedback

If you have any feedback or suggestions about this project, please open an [issue](https://github.com/aiulus/ddsf/issues) or contact [aybuke.ulusarslan@tum.de].


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

