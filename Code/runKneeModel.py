"""
Author:             FJ van Melis, Sabrina Hörmann
Created on:         September 13th 2024.
Last updated on:    April 8th 2026.

PURPOSE:
Run RMR_solver solver.

INPUT FILES (../Input/):
    - K8L_RMR_scaled.osim  : Scaled OpenSim model
    - grf.mot              : Ground reaction forces
    - motion.mot           : IK motion output
    - emg.csv              : EMG data (required if CCI_informed or EMG_informed)

OUTPUT FILES (../Output/):
    - joint_forces.csv
    - muscle_activation.csv
    - reserve_forces.csv
"""

import logging
import warnings
from pathlib import Path

import numpy as np
import opensim

import utilsLoadFile
from RMRsolver import RMRsolver
from PlayBackActive import PlayBackActive
from MotionAnalysis import MotionAnalysis
from PlayBackEMG import PlayBackEMG
import utilsObjectives as utilsObj
from utilsRunRMR import *

warnings.filterwarnings('ignore', category=RuntimeWarning)
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


def run_knee_model(
    CCI_informed: bool = False,
    EMG_informed: bool = True,
    EMG_keywords: list = None,
    output_joints: list = None,
    agonist: str = None,
    antagonist: str = None,
) -> None:
    """
    Run the RMR solver for the knee musculoskeletal model.

    Args:
        CCI_informed:   Use CCI-informed objective. Mutually exclusive with EMG_informed.
        EMG_informed:   Use EMG-informed objective. Mutually exclusive with CCI_informed.
        EMG_keywords:   Muscle name substrings for EMG tracking. Only used if EMG_informed=True.
        output_joints:  Joint names to extract forces for.
        agonist:        Agonist muscle for CCI objective. Only used if CCI_informed=True.
        antagonist:     Antagonist muscle for CCI objective. Only used if CCI_informed=True.
    """
    if CCI_informed and EMG_informed:
        raise ValueError("CCI_informed and EMG_informed cannot both be True.")
    # Define agonist and antagonist muscle for CCI informed simulations
    if CCI_informed:
        agonist = 'gasmed_l'
        antagonist = 'vaslat_l'
    if EMG_keywords is None:
        EMG_keywords = ['recfem', 'vaslat', 'vasmed', 'bflh', 'gas', 'semiten', 'tibant'] # define all the muscles where EMG measurements are available
    if output_joints is None:
        output_joints = ['walker_knee_l', 'walker_knee_r'] # define for which joints the output should be saved

    base = Path(__file__).resolve().parent.parent
    path_model = base / "Input" / "knee_model" / "K8L_RMR_scaled.osim"
    path_ef    = base / "Input" / "knee_model" / "grf.mot"
    path_mot   = base / "Input" / "knee_model" / "motion.mot"

    for p in [path_model, path_ef, path_mot]:
        if not p.exists():
            raise FileNotFoundError(f"Missing input file: {p}")

    # Load model
    model = opensim.Model(str(path_model))
    force_storage = opensim.Storage(str(path_ef), False)  # keep in scope

    # Define external force and add to the model
    ef = opensim.ExternalForce()
    ef.setName('Ground_reaction_force')
    ef.set_applied_to_body('calcn_l')
    # adapt the settings based on the input data (grf.mot file)
    ef.set_force_expressed_in_body('ground')
    ef.set_point_expressed_in_body('ground')
    ef.set_force_identifier('2_ground_force_v')
    ef.set_point_identifier('2_ground_force_p')
    ef.set_torque_identifier('ground_torque_')
    ef.set_data_source_name(str(path_ef.resolve()))
    ef.setDataSource(force_storage)
    model.addForce(ef)

    # Load motion and filter
    motion = PlayBackActive(model, str(path_mot), path_mot.name, cartesians=['pelvis_tx', 'pelvis_ty', 'pelvis_tz']) # include the coordinates in cartesian
    motion.filter(hz=6)
    motion.setTime(hz=99)

    # Objective function weights:
    # Muscles are assigned a default weight of 1.
    # Coordinate actuators receive:
    #   - weight = 0 if no muscles are available to actuate the coordinate,
    #   - weight = 10 if the coordinate should be primarily driven by muscles.
    muscles   = model.getMuscles()
    actuators = model.getActuators()
    muscle_names   = {muscles.get(i).getName() for i in range(muscles.getSize())}
    joint_keywords = ['hip', 'ankle', 'mtp', 'subtalar'] # Specify which joints are actuated by muscles

    # Define actuator weights
    weights = np.array([
        1  if actuators.get(i).getName() in muscle_names else
        10 if actuators.get(i).getName() in ('knee_angle_l', 'knee_angle_r')
           or any(kw in actuators.get(i).getName() for kw in joint_keywords) else
        0
        for i in range(actuators.getSize())
    ])

    # Build the objective function; its formulation depends on whether the case is CCI-informed, EMG-informed, or minimum activation
    if CCI_informed:
        names = [actuators.get(i).getName() for i in range(actuators.getSize())]
        objective = utilsObj.ActSquared_CCItrack(weights, 2, names.index(agonist), names.index(antagonist)) # CCI weight = 2
    elif EMG_informed:
        weights_EMG = np.array([
            2 if any(kw in actuators.get(i).getName() for kw in EMG_keywords) else 0 #EMG weight =2
            for i in range(actuators.getSize())
        ])
        objective = utilsObj.ActSquared_EMGtrack(weights, weights_EMG)
    else:
        objective = utilsObj.ActSquared(weights)

    solver = RMRsolver(model, solveInfo=True, constrainActDyn=True, constrainGHjoint=False,
                       outputJoint=output_joints, visualize=False,
                       actuatorReserveBounds=[-2000, 2000], actuatorDelta=[0.001, 0.005])
    solver.setObjective(objective)
    solver.info()

    if CCI_informed or EMG_informed:
        path_emg = base / "Input" / "knee_model" / "emg.csv" # include EMG measurements
        analysis = MotionAnalysis(motion=motion, solver=solver, EMGplayer=PlayBackEMG(path=str(path_emg)))
    else:
        analysis = MotionAnalysis(motion=motion, solver=solver)

    result = analysis.runAll()
    analysis.addPlot('info')
    # analysis.addPlot('reserve')
    # analysis.addPlot('coordinate')
    analysis.addPlot('joint')
    analysis.plotAll([result])
    # save results to a .csv file
    analysis.saveToCSV(
        base / "Output" / "knee_model" / "joint_forces.csv",
        base / "Output" / "knee_model" / "muscle_activation.csv",
        base / "Output" / "knee_model" / "reserve_forces.csv",
        [result], labels=['Estimation']
    )
    logger.info("Done. Results saved to %s", base / "Output")


if __name__ == "__main__":
    run_knee_model()