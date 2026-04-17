# rmr-solver-py

**rmr-solver-py** is a Python implementation of the **Rapid Muscle Redundancy (RMR) solver** — an algorithm that solves the muscle redundancy problem by determining which muscles the human body recruits to produce a given motion, using a musculoskeletal model.

This repository extends the original MATLAB RMR solver with two approaches:
- **EMG-informed** muscle activation estimation
- **Co-contraction Index (CCI)-informed** muscle activation estimation

Both approaches account for changes in muscle coordination strategy, enabling more physiologically realistic simulations.

> The original MATLAB implementation is available at: [ComputationalBiomechanicsLab/rmr-solver](https://github.com/ComputationalBiomechanicsLab/rmr-solver)

---

## Publications

If you use this solver, please cite the original RMR paper:

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

For the EMG-informed and CCI-informed extensions, please also cite:

```bibtex
@article{hoermann2026,
  title={Co-contraction index informed simulations capture muscle coordination strategies driven changes in compressive knee joint contact forces in musculoskeletal modeling},
  author={Hörmann, Sabrina and Tumer, Nazli and Zadpoor, Amir A. and Seth, Ajay},
  journal={X},
  volume={X},
  number={X},
  pages={X},
  year={X},
  publisher={X}
}
```

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

---

## Contributors

- Sabrina Hörmann
- Florian van Melis

---

## Acknowledgements

This work is part of the **LoaD project** (project no. NWA1389.20.009) within the NWA-ORC research programme, (partly) financed by the **Dutch Research Council (NWO)**.

---

## License

This code is licensed under the **Apache 2.0 License** — see the [`LICENSE`](LICENSE) file for details.
