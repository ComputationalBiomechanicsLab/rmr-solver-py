"""
Author:             FJ van Melis
Created on:         September 13th 2024.
Last updated on:    December 4th 2024.

PURPOSE:
Run RMR_solver solver on single motion and compare to EMG data.
"""

import os
import warnings
import numpy as np
from pathlib import Path

import utilsLoadFile
from RMRsolver import RMRsolver
from PlayBackActive import PlayBackActive
from MotionAnalysis import MotionAnalysis
from PlayBackEMG import PlayBackEMG
import utilsObjectives as utilsObj
import opensim
from utilsRunRMR import *
warnings.filterwarnings('ignore', category=RuntimeWarning)

# NOTE: Select path to OpenSim model (.osim):
base = Path(__file__).resolve().parent.parent
pathModel = base / "Input" / "shoulder_model" / "TSM_Ajay2019_noWeight.osim"
path_mot   = base / "Input" / "shoulder_model" / "flx01_IK.mot"

# Initialize classes:
model = opensim.Model(str(pathModel))
motion = PlayBackActive(model= model, path= str(path_mot),name= path_mot.name)
#EMGplayer = PlayBackEMG(folder= folderEMG, EMGname= nameEMG, MVCname= 'MVC')

# NOTE: Set motion filter and crop options:
motion.filter(hz=3)
motion.setTime(0.637, 7.187,hz=10) # recommended time ranges in this example: flx01:  0.637, 7.187 | abd01: 3.043, 10.875

# NOTE: Set RMR_solver solver options:
weights = np.concatenate( (np.ones(33), np.zeros(8), 10*np.ones(9)) )
objective = utilsObj.ActSquared(weights)
solver = RMRsolver(model,solveInfo=True,constrainActDyn=True,constrainGHjoint=True,outputJoint=['GlenoHumeral','scapulothoracic'],visualize=False)
solver.setObjective(objective)

solver.info()

"""
`weights` represents the weight factors used in the objective function corresponding to the actuators.

This is dependent on the model used and the intentions of the user. A good start is weights `1.0` for all muscles, 
`0.0` for ground actuators or irrelevant reserve coordinate coordinates and `10.0` for relevant coordinate reserve actuators.

Use `solver.info()` to view the list of actuators (in order) of the selected OpenSim model.
"""

# NOTE: Set motion analysis options
analysis = MotionAnalysis(motion= motion, solver= solver)

analysis.addPlot('coordinate',plotRange= ['plane_elv','shoulder_elv','axial_rot']) # coordinate values over time
analysis.addPlot('activation') # muscle activation over time
analysis.addPlot('info') # solver info
analysis.addPlot('GHjoint') # Glenohumeral joint reaction force magnitude and angle over time
analysis.addPlot('reserve') # reserve actuator activation over time
analysis.addPlot('joint') # joint reaction forces expressed in ground frame over time

result = analysis.runAll()
analysis.plotAll([result], labels=['Estimation'])
