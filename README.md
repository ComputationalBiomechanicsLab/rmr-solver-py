# rmr-solver-py

**rmr-solver-py** is a Python implementation of the **Rapid Muscle Redundancy (RMR) solver** — an algorithm that solves the muscle redundancy problem by determining which muscles the human body recruits to produce a given motion, using a musculoskeletal model.

This repository extends the original MATLAB RMR solver with two approaches:
- **EMG-informed** muscle activation estimation
- **Co-contraction Index (CCI)-informed** muscle activation estimation

Both approaches account for changes in muscle coordination strategy, enabling more physiologically realistic simulations.
Further details are described in this paper:

>**Paper**: S. Hörmann, N. Tumer, A. A. Zadpoor, and A. Seth, "Co-Contraction Index Informed Simulations Capture Muscle Coordination Strategies Driven Changes in Compressive Knee Joint Contact Forces in Musculoskeletal Modeling," X, 2026.

The original MATLAB implementation is available at: [ComputationalBiomechanicsLab/rmr-solver](https://github.com/ComputationalBiomechanicsLab/rmr-solver)



---

## Data

This work uses the publicly available **Comprehensive Assessment of the Musculoskeletal System (CAMS) Knee Dataset**.

Access the dataset at: [orthoload.com/cams-knee-project-online](https://orthoload.com/cams-knee-project-online/)

---

## Installation

```bash
pip install -r requirements.txt
```

---

## Repository Structure

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
│   │   ├── emg.csv
│   │   ├── grf.mot
│   │   └── motion.mot
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

## Getting Started

### Entry Point Scripts

| Script | Description |
|--------|-------------|
| `runKneeModel.py` | Executes the full pipeline for the knee model simulation |
| `runShoulderModel.py` | Executes the full pipeline for the shoulder model simulation |

### Input Data

#### Knee Model (`Input/knee_model/`)
- **OpenSim model:** `K8L_RMR_scaled.osim`
- **Geometry:** `geometry/`
- **From CAMS dataset** *(download separately)*:
  - Motion data: `motion.mot`
  - Ground reaction forces: `grf.mot`
  - EMG data: `emg.csv`

#### Shoulder Model (`Input/shoulder_model/`)
- **OpenSim model:** `TSM_Ajay2019_noWeight.osim`
- **Kinematic trials:** `abd01_IK.mot`, `flx01_IK.mot`
- **Geometry:** `Geometry/`

### Output Data

The output of the RMR solver includes:
- joint_forces.csv: The resulting joint reaction forces in N and moment in N/m2 of the defined output joint. The results are reported in the child (_child) and ground (_ground) reference frames. Furthermore the csv fiels includes a column on the time steps. 
- muscle_activations.csv: The csv files includes the calculcated muscle activations for each muscle in the model.
- reserve_forces.csv: Here we report the resulting reserve forces of the reserve actuators.
- 
---

## Contributors

- Sabrina Hörmann
- Florian van Melis

---

## Publications

If you use this solver, please cite the original RMR paper:

> **Paper:** I. Belli, S. Joshi, J. M. Prendergast, I. Beck, C. Della Santina, L. Peternel, and A. Seth, "Does Enforcing Glenohumeral Joint Stability Matter? A New Rapid Muscle Redundancy Solver Highlights the Importance of Non-Superficial Shoulder Muscles," *PLOS ONE*, vol. 18, no. 11, p. e0295003, 2023.

For the EMG-informed and CCI-informed extensions, please also cite:

>**Paper**: S. Hörmann, N. Tumer, A. A. Zadpoor, and A. Seth, "Co-Contraction Index Informed Simulations Capture Muscle Coordination Strategies Driven Changes in Compressive Knee Joint Contact Forces in Musculoskeletal Modeling," X, 2026.


## Acknowledgements

This work is part of the **LoaD project** (project no. NWA1389.20.009) within the NWA-ORC research programme, (partly) financed by the **Dutch Research Council (NWO)**.

---

## License

This code is licensed under the **Apache 2.0 License** — see the [`LICENSE`](LICENSE) file for details.
