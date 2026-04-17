"""
Author:             FJ van Melis, Sabrina Hörmann
Created on:         October 18th 2024.
Last updated on:    April 8th 2026.

PURPOSE:
Class setting the objective for the optimization problem.

USAGE:
See runRMRsolver().py for an example on how to use this class!
"""

from typing import Any, List, Dict, Tuple, Optional, Union
import numpy as np
import numpy.typing as npt


class ActSquared:
    def __init__(self, actuatorWeights: npt.ArrayLike) -> None:
        """
        Sets up an objective function based on minimization of activation squared.

        :Parameters:
        actuatorWeights: ``array_like`` |
            Weights corresponding to the actuators of the model.
        """
        self.weight = actuatorWeights

    def __call__(self, x: npt.ArrayLike, *args) -> Any:
        """
        Callable cost function for scipy.minimize() in RMR_solver solver.
        """
        cost = self.weight.dot(np.square(x))

        return cost

    def getWeight(self) -> npt.NDArray:
        return self.weight

class ActSquared_EMGtrack:
    def __init__(self, actuatorWeights: npt.ArrayLike, emgWeights: npt.ArrayLike) -> None:
        """
        Sets up an objective function based on minimization of activation squared and error in
        muscle activation (normalized EMG - simulated muscle activation) squared.

        :Parameters:
        actuatorWeights: ``array_like`` |
            Weights corresponding to the actuators of the model.
        EMGWeights: ``array_like`` |
            Weights corresponding to the error in muscle activation of the model.
        """
        self.weight = actuatorWeights
        self.weight_EMG = emgWeights

    def __call__(self, x: npt.ArrayLike, EMG: npt.ArrayLike) -> Any:
        """
        Callable cost function for scipy.minimize() in RMR_solver solver.
        """
        EMGdiff = np.abs(x - EMG)

        cost = self.weight.dot(np.square(x)) + self.weight_EMG.dot(np.square(EMGdiff))

        return cost

    def getWeight(self) -> npt.NDArray:
        return self.weight


class ActSquared_CCItrack:
    def __init__(self, actuatorWeights: npt.ArrayLike, CCIWeights: npt.ArrayLike, agonist_idx: int, antagonist_idx: int ) -> None:
        """
        Sets up an objective function based on minimization of activation squared and of error in muscle co-contraction squared.
        Co-contraction index is calculated from gastroc medialis and vastus lateralis following the definition from Rudolph et al. 2000,

        :Parameters:
        actuatorWeights: ``array_like`` |
            Weights corresponding to the actuators of the model.
        CCIWeights: ``array_like`` |
            Weights corresponding to the error in muscle activation of the model.
        """
        self.weight = actuatorWeights
        self.weight_CCI = CCIWeights
        self.agonist_idx = agonist_idx
        self.antagonist_idx = antagonist_idx

    def __call__(self, x: npt.ArrayLike, EMG: npt.ArrayLike) -> Any:
        """
        Callable cost function for scipy.minimize() in RMR_solver solver.
        """

        agonist =  x[self.agonist_idx]
        antagonist = x[self.antagonist_idx]
        if agonist >= antagonist:
            CCI_model = antagonist / agonist * (antagonist + agonist)
        else:
            CCI_model = agonist / antagonist * (antagonist + agonist)
        agonist_EMG =  EMG[self.agonist_idx]
        antagonist_EMG = EMG[self.antagonist_idx]
        if agonist_EMG > antagonist_EMG:
            CCI_EMG = antagonist_EMG / agonist_EMG * (antagonist_EMG + agonist_EMG)
        else:
            CCI_EMG = agonist_EMG / antagonist_EMG * (antagonist_EMG + agonist_EMG)

        CCIdiff = np.abs(CCI_EMG - CCI_model)

        cost = self.weight.dot(np.square(x)) + self.weight_CCI * np.square(CCIdiff)

        return cost

    def getWeight(self) -> npt.NDArray:
        return self.weight

