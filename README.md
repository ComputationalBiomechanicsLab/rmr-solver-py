# rmr-solver-py

RMR solver stands for Rapid Muscle Redundancy solver. It is an algorithm which solves the muscle redundancy problem, by selecting the muscles which are recruited by the human body to generate a given motion (leveraging a musculoskeletal model). The solver is presented in detail in this paper:

```bibtex
@article{belli2023does,
  title={Does enforcing glenohumeral joint stability matter? A new rapid muscle redundancy solver highlights the importance of non-superficial shoulder muscles},
  author={Belli, Italo and Joshi, Sagar and Prendergast, J Micah and Beck, Irene and Della Santina, Cosimo and Peternel, Luka and Seth, Ajay},
  journal={Plos one},
  volume={18},
  number={11},
  pages={e0295003},
  year={2023},
  publisher={Public Library of Science San Francisco, CA USA}
}
```
This repository includes the rmr-solver in python. The orginal matlab version can be find in this repsository: https://github.com/ComputationalBiomechanicsLab/rmr-solver.git

In this work, the original rmr-sovler got extended by the EMG-informed and co-contraction index (CCI)-informed approach. 
Both approaches aim to take into account changes in muscle coordination strategy when estimating muscle activations. 
Further details can be found in our paper:

```bibtex
@article{hoermann2026,
  title={Co-contraction index informed simulations capture
muscle coordination strategies driven changes in
compressive knee joint contact forces in
musculoskeletal modeling},
  author={Hörmann, Sabrina and Tumer, Nazli and Zadpoor, Amir A. and Seth, Ajay},
  journal={X},
  volume={X},
  number={X},
  pages={X},
  year={X},
  publisher={X}
}
```
# Data
In this paper, we are using the publicly available Comprehensive Assessment of the Musculoskeletal System
(CAMS) knee dataset.
You can access the data through this website: https://orthoload.com/cams-knee-project-online/

# Installation

```bash
pip install -r requirements.txt
```

# Structure

```
.
├── Code/
│   ├── MotionAnalysis.py
│   ├── PlayBackActive.py
│   ├── PlayBackEMG.py
│   ├── RMRsolver.py
│   ├── runKneeModel.py
│   ├── runShoulderModel.py
│   ├── utilsLoadFile.py
│   ├── utilsObjectives.py
│   └── utilsRMRsolver.py
│
├── Input/
│   ├── knee_model/
│   │   ├── geometry/
│   │   ├── K8L_RMR_scaled.osim
│   │
│   └── shoulder_model/
│       ├── Geometry/
│       ├── TSM_Ajay2019_noWeight.osim
│       ├── abd01_IK.mot
│       ├── flx01_IK.mot
│       └── opensim.log
│
├── .gitignore.txt
├── LICENSE
├── README.md
└── requirements.txt
```

---
# Getting started 

### Core Scripts
- `runKneeModel.py`  
  Executes the full pipeline for the knee model simulation.

- `runShoulderModel.py`  
  Runs the shoulder musculoskeletal model pipeline.
  
## Input Data

### Knee Model (`Input/knee_model/`)
- OpenSim model: `K8L_RMR_scaled.osim`
- Geometry assets in `geometry/`

Please download the following data from the CAMS dataset website
- Motion data: `motion.mot`
- Ground reaction forces: `grf.mot`
- EMG data: `emg.csv`

### Shoulder Model (`Input/shoulder_model/`)
- OpenSim model: `TSM_Ajay2019_noWeight.osim`
- Kinematic trials: `abd01_IK.mot`, `flx01_IK.mot`
- Geometry assets in `Geometry/`
  
# Contributors
Sabrina Hörmann, Florian van Melis

# Acknowlegement
This work is part of the project LoaD (projectnr. NWA1389.20.009) of the NWA-ORC research programme which is
(partly) financed by the Dutch Research Council (NWO).

# License
Our code is licensed under the Apache 2.0 license (see the LICENSE_code file). 
