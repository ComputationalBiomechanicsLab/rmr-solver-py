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

# Data

# Requirements

# Acknowlegement
This work is part of the project LoaD (projectnr. NWA1389.20.009) of the NWA-ORC research programme which is
(partly) financed by the Dutch Research Council (NWO).

# License
