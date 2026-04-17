"""
Based on original method by I. Belli and Python adaptation from I. Beck.

Author:             FJ van Melis, Sabrina Hörmann
Created on:         September 13th 2024.
Last updated on:    April 10th 2026.

PURPOSE:
Class for perparing a Rapid Muscle Redundancy (RMR_solver) solver for use, by providing an OpenSim model and configuring settings.

USAGE:
See runRMRsolver().py for an example on how to use this class!

_init_: 
    Set up RMR_solver solver by providing OpenSim model and optionally change solver and constraint settings.
setObjective: 
    Set up RMR_solver solver objective (otherwise default is selected upon first solve).
info(): 
    Print RMR_solver solver info to terminal.
solve(): 
    Solve for a single time instant by provide kinematic state (preferred use with MotionAnalysis class).

"""
from typing import Any, List, Dict, Tuple, Optional, Union
import opensim
import numpy as np
import numpy.typing as npt
from dataclasses import dataclass, field
import scipy.optimize as sopt
from datetime import datetime

import utilsRMRsolver as utilsRMR
import utilsObjectives as utilsObj

class RMRsolver:
    def __init__(self, model: opensim.Model, **settingsOverwrite) -> None:
        """
        Sets up RMR_solver solver for selected OpenSim model. Change settings from default by providing keyword arguments.

        :Parameters:
        model: ``opensim.Model``
            OpenSim model to use.

        constrainActDyn: ``bool`` (default = True)
            Toggle muscle activation dynamics constraint.

        constrainGHjoint: ``bool`` (default = False)
            Toggle glenohumeral joint reaction force constraint (requires suitable Delft Shoulder Elbow Model).

        outputJoint: ``list[str]`` (default = empty)
            Names of the joints for which to output reaction forces (expressed in ground frame)

        visualize: ``bool`` (default = False)
            Toggle external window with visualization of the model movement during solving.
        """

        self.model = model

        # Set scipy.optimize.minimize() settings:
        self.solverSettings = dict(maxiter=10000,
                                   ftol= 1E-6,
                                   disp= False,
                                   eps= 1E-8)
        
        # Set default settings and overwrite if applicable:
        self.settings = dict(actuatorMuscleBounds=[0,1], # activation bounds
                        actuatorMuscleGuess=0.1, # activation initial guess
                        actuatorReserveBounds=[-600,600],
                        actuatorReserveGuess=0,
                        actuatorDelta=[0.001,0.005], # tolerance on acceleration constraint
                        solveInfo=False, # print solve info to terminal
                        constrainActDyn=True, # constrain activation dynamics
                        tauAct=0.01, # activation time constant
                        tauDeact=0.04, # de-activation time constant
                        constrainGHjoint=False, # constrain glenohumeral joint
                        visualize=False, # visualize motion during solving
                        outputJoint=[]) # list of joint names for which to output force

        self.settings.update(settingsOverwrite)

        # Set up coordinates:
        self.coordinateSet = self.model.getCoordinateSet()
        self.coordinateNum = self.coordinateSet.getSize()
        self.coordinates = []
        self.coordinateNames = []
        for i in range(self.coordinateNum):
            self.coordinates.append(self.coordinateSet.get(i))
            self.coordinateNames.append(self.coordinates[i].getName())
        
        # Set up muscles:
        muscleSet = self.model.getMuscles()
        self.muscleNum = muscleSet.getSize()
        self.muscles = []
        self.muscleNames = []
        self.muscleForceMax = []

        for i in range(self.muscleNum):
            self.muscles.append(opensim.Millard2012EquilibriumMuscle.safeDownCast(muscleSet.get(i)))
            self.muscleNames.append(self.muscles[i].getName())
            self.muscles[i].set_ignore_tendon_compliance(True)
            self.muscles[i].set_ignore_activation_dynamics(True)
            self.muscleForceMax.append(self.muscles[i].getMaxIsometricForce())

        # Initialize system:
        if self.settings['visualize']:
            self.model.setUseVisualizer(True)

        self.model.finalizeConnections()
        self.state = self.model.initSystem()

        # Set up joints:
        outputJoint = self.settings['outputJoint']
        if not len(outputJoint) == 0:
            jointSet = self.model.getJointSet()
            self.joints = []
            for name in outputJoint:
                self.joints.append( jointSet.get(name))
            self.jointNum = len(self.joints)
        else:
            self.jointNum = 0

        if self.settings['constrainGHjoint']:
            jointSet = self.model.getJointSet()
            self.jointGH = jointSet.get('GlenoHumeral')
            self.jointGHmaxAngle = utilsRMR.get_glenoid_status(self.model, self.state)[0]
        else:
            self.jointGHmaxAngle = 0

        # Set up coordinate actuators:
        actuatorSet = self.model.getActuators()
        self.actuatorNum = actuatorSet.getSize()
        self.actuators = []
        self.actuatorNames = []
        self.actuatorMuscleIndex = [] # indices of actuators which are muscles

        for i in range(self.actuatorNum):
            self.actuatorNames.append(actuatorSet.get(i).getName())
            self.actuators.append(opensim.ScalarActuator.safeDownCast(actuatorSet.get(i)))
            if self.actuatorNames[i] in self.muscleNames: # only allow overwrite for muscle actuators
                self.actuatorMuscleIndex.append(i)
                self.actuators[i].overrideActuation(self.state, True)
        
        # Set up bounds for actuator controls:
        self.xBounds = []
        for i in range(self.actuatorNum):
            if i in self.actuatorMuscleIndex:
                self.xBounds.append( (self.settings['actuatorMuscleBounds'][0], self.settings['actuatorMuscleBounds'][1]) )
            else:
                self.xBounds.append( (self.settings['actuatorReserveBounds'][0], self.settings['actuatorReserveBounds'][1]) )
        
        # Set up initial settings:
        self.x0 = np.concatenate((self.settings['actuatorMuscleGuess'] * np.ones(self.muscleNum),
                                  self.settings['actuatorReserveGuess'] * np.ones(self.actuatorNum - self.muscleNum)))
        self.xDelta = self.settings['actuatorDelta']
        self.init = True

        # Fetch optimal forces reserve actuators:
        self.actuatorReserveOptimalForce = np.zeros(self.actuatorNum - self.muscleNum)

        for i, act in enumerate(self.actuators[self.muscleNum:]):
            self.actuatorReserveOptimalForce[i] = act.getOptimalForce()

        self.hasObjective = False

    def setObjective(self, objective: utilsObj.ActSquared) -> None:
        """
        Initialize objective function.
        """
        self.objective = objective
        weightNum = len( objective.getWeight())
        if not weightNum == self.actuatorNum:
            raise AssertionError(f"Number of weights of the objective is not equal to the number of actuators ({weightNum} and {self.actuatorNum})")

        self.hasObjective = True

    def info(self) -> None:
        """ 
        Print basic info of the RMR_solver solver to terminal.
        """

        print(">>> INFO: RMR_solver SOLVER REPORT\n")

        print(f"Number of coordinates = {self.coordinateNum}")
        print(f"Number of muscles = {self.muscleNum}")
        print(f"Number of actuators = {self.actuatorNum}")

    
        print("\n--- COORDINATES ---")
        for i, item in enumerate(self.coordinateNames):
            print(f"{i}. {item}")

        print("\n--- MUSCLES ---")
        for i, item in enumerate(self.muscleNames):
            print(f"{i}. {item}")

        print("\n--- ACTUATORS ---")
        for i, item in enumerate(self.actuatorNames):
            print(f"{i}. {item}")

        print("\n--- JOINTS ---")
        jointSetTotal = self.model.getJointSet()
        jointNumTotal = jointSetTotal.getSize()
        for i in range(jointNumTotal):
            print(f"{i}. {jointSetTotal.get(i).getName()}")
        


    def solve(self, time: float, position: npt.ArrayLike, speed: npt.ArrayLike, acceleration: npt.ArrayLike, EMG: Optional[npt.ArrayLike] = None) -> Tuple[npt.NDArray, npt.NDArray, npt.NDArray, Dict]:
        """ 
        Solve for state and return activations.
        """

        timeStart = datetime.now()

        # Set default objective if it is missing:
        if not self.hasObjective:
            raise RuntimeError("No objective was set for RMR_solver solver")

        if EMG is None:
            EMG = 0

        # Add this right after the activation dynamics bounds update section
        if self.settings['constrainActDyn'] and not self.init:
            timeStep = time - self.timePrev

            for i in range(self.muscleNum):
                actuator_idx = self.actuatorMuscleIndex[i]

                # Calculate bounds (same as your existing code)
                lb = np.max((self.x0[actuator_idx] - self.x0[actuator_idx] * (
                            0.5 + 1.5 * self.x0[actuator_idx]) * timeStep / self.settings['tauDeact'], 0))
                ub = np.min((self.x0[actuator_idx] + (1 - self.x0[actuator_idx]) * timeStep / (
                            self.settings['tauAct'] * (0.5 + 1.5 * self.x0[actuator_idx])), 1))

                # Update bounds (your existing code)
                self.xBounds[actuator_idx] = (lb, ub)

  
        # Update activation bounds if enabled (constraint 2):
        if self.settings['constrainActDyn'] and not self.init:
            timeStep = time - self.timePrev

            for i in range(self.muscleNum):
                lb = np.max((self.x0[i] - self.x0[i] * (0.5 + 1.5 * self.x0[i]) * timeStep / self.settings['tauDeact'], 0))
                ub = np.min((self.x0[i]  + (1 - self.x0[i] ) * timeStep / (self.settings['tauAct'] * (0.5 + 1.5 * self.x0[i] )), 1))

                self.xBounds[self.actuatorMuscleIndex[i]] = (lb, ub)

        self.timePrev = time
        self.init = False # next time trigger activation bounds update

        # Update state:
        self.state.setTime(time)

        for i, coord in enumerate(self.coordinates):
            coord.setValue(self.state, position[i], False)
            coord.setSpeedValue(self.state, speed[i])

        self.model.assemble(self.state)
        self.model.equilibrateMuscles(self.state)
        self.model.realizeVelocity(self.state)
        modelControls = self.model.getControls(self.state)

        # Get muscle force multipliers:
        muscleForceActiveMax = []
        muscleForcePassiveMax = []
        muscleMomentArm = []

        for i, muscle in enumerate(self.muscles):
            ForceLen = muscle.getActiveForceLengthMultiplier(self.state)
            ForceVel = muscle.getForceVelocityMultiplier(self.state)
            ForcePas = muscle.getPassiveForceMultiplier(self.state)
            cosPenn = muscle.getCosPennationAngle(self.state)

            muscleForceActiveMax.append(ForceLen * ForceVel * self.muscleForceMax[i] * cosPenn)
            muscleForcePassiveMax.append(ForcePas * self.muscleForceMax[i] * cosPenn)

            for j, coord in enumerate(self.coordinates):
                muscleMomentArm.append([])
                muscleMomentArm[j].append(muscle.computeMomentArm(self.state, coord))

        # Set up acceleration constraint (constraint 1):
        @dataclass
        class paramsAcc:
            _model: Any = self.model
            _state: Any = self.state
            _actuators: list = field(default_factory=list)
            _actuatorNames: list = field(default_factory=list)
            _coordinates: list = field(default_factory=list)
            _coordinateNames: list = field(default_factory=list)
            _coordinateNum: int = self.coordinateNum
            _muscles: list = field(default_factory=list)
            _muscleNames: list = field(default_factory=list)
            _muscleNum: int = self.muscleNum
            _useMuscles: bool = True
            _useControls: bool = True
            _forceActive: list = field(default_factory=list)
            _forcePassive: list = field(default_factory=list)
            _modelControls: Any = modelControls

        paramsAcc._actuators = self.actuators
        paramsAcc._actuatorNames = self.actuatorNames
        paramsAcc._coordinates = self.coordinates
        paramsAcc._coordinateNames = self.coordinateNames
        paramsAcc._muscles = self.muscles
        paramsAcc._muscleNames = self.muscleNames
        paramsAcc._forceActive = muscleForceActiveMax
        paramsAcc._forcePassive = muscleForcePassiveMax

        if self.settings['constrainGHjoint']:
            paramsAcc._GHjoint = self.jointGH

            constrAcc_A, constrAcc_b, constrGH_A, JRFnoAct = utilsRMR.constructAccelerationConstraint_GH(paramsAcc, acceleration, self.actuatorNum, self.coordinateNum)

            # Set up GH-JRF constraint (constraint 3):
            Vec_H2GC = utilsRMR.get_glenoid_status(self.model, self.state)[1]

            @dataclass
            class paramsGH:
                _vectorGlenoid: Any = Vec_H2GC
                _JRFnoAct: np.array = JRFnoAct
                _A: Any = constrGH_A
                _maxAngle: float = self.jointGHmaxAngle

            constraintGH = sopt.NonlinearConstraint(lambda x: utilsRMR.GHjointConstraint(x, paramsGH), -1.0, 0.0)

        else:
            constrAcc_A, constrAcc_b = utilsRMR.constructAccelerationConstraint(paramsAcc, acceleration, self.actuatorNum, self.coordinateNum)

        # Employ solver: (retry for higher tolerance xDelta if optimization is unsuccessful)
        for xDelta in self.xDelta:
            xDeltaUsed = xDelta # save xDelta used for printing/plotting

            # Finalize constraints:
            constrAcc_b_lb, constrAcc_b_ub = utilsRMR.addTolerance(vector=constrAcc_b, tolerance= xDelta)
            
            constraintAcc = sopt.LinearConstraint(A= constrAcc_A, lb=constrAcc_b_lb, ub=constrAcc_b_ub, keep_feasible=False)

            if self.settings['constrainGHjoint']:
                constraints = [constraintAcc, constraintGH]
            else:
                constraints = [constraintAcc]

            # Solve:
            result = sopt.minimize(self.objective, 
                                    x0= self.x0,
                                    args= EMG,
                                    method='SLSQP', 
                                    constraints=constraints, 
                                    bounds=self.xBounds,
                                    options=self.solverSettings)
            if result.success:
                break

            self.x0 = result.x # better initial guess for next try
        
        # Get results:
        timeElapsed = datetime.now() - timeStart 
        self.x0 = result.x # better initial guess for next time instant
        activation = result.x

        # Get cost components from objective function
        cost_components = {}
        if hasattr(self.objective, 'get_cost_components'):
            cost_components = self.objective.get_cost_components()

        # activeAcc = utilsRMR.getInducedAcceleration(activation, paramsAcc)
        # noActAcc = utilsRMR.getInducedAcceleration(np.zeros(self.actuatorNum), paramsAcc)
        # constraintViolation = activeAcc - noActAcc - constrAcc_b

        constraintViolation = constrAcc_A.dot(activation) - constrAcc_b
        constraintViolationRelative = constraintViolation/constrAcc_b/xDeltaUsed

        # Create results dictionary
        results_dict = {
            'total_cost': result.fun,
            'success': result.success,
            'message': result.message if not result.success else "Optimization succeeded",
            'xDelta': xDeltaUsed,
            'time_elapsed': timeElapsed,
            'constraint_violation': constraintViolation,
            'constraint_violation_relative': constraintViolationRelative,
            **cost_components  # Unpack cost components from objective
        }
        # Print solver log if enabled
        if self.settings['solveInfo']:
            if result.success:
                print(">>> INFO: Optimization succeeded:")
            else:
                print(">>> INFO: Optimization failed:")
                print(result.message)

            print(f"Final cost = {result.fun}")
            print(f"xDelta = {xDeltaUsed}")
            print(f"Time elapsed = {timeElapsed}")

        # Acquire joint reaction forces for joints specified in outputJoints:
        if not self.jointNum == 0:
            # Initialize the muscles to produce the required forces
            forceTotal = muscleForceActiveMax * activation[:self.muscleNum] + muscleForcePassiveMax
            for i, muscle in enumerate(self.muscles):
                muscle.setOverrideActuation(self.state, forceTotal[i])

            # Initialize the CoordinateActuators to produce the required effect
            for i, act in enumerate(self.actuators):
                if not self.actuatorNames[i] in self.muscleNames:
                    act.setControls(opensim.Vector(1, activation[i]), modelControls)
            
            # Realize the model to the acceleration stage:
            self.model.realizeVelocity(self.state)
            self.model.setControls(self.state, modelControls)
            self.model.realizeAcceleration(self.state)

            #report the joint forces in the child frame and the ground frame
            jointForcesChild = np.zeros([self.jointNum, 6])
            jointForcesGround = np.zeros([self.jointNum, 6])
            for i, joint in enumerate(self.joints):
                child_body_name = joint.getChildFrame().findBaseFrame().getName()
                child_body_frame = self.model.getComponent(f"bodyset/{child_body_name}") # in our study Hörmann et al. 2027 we accounted for the tibial slope and reported the output in 'bodyset/{child_body_name}/{child_body_name}_offsetframe'

                jointForcesChild[i, :3] = self.model.getGround().expressVectorInAnotherFrame(self.state,
                                                                                         joint.calcReactionOnChildExpressedInGround(
                                                                                             self.state).get(0),
                                                                                         child_body_frame).to_numpy()  # moments
                jointForcesChild[i, 3:] = self.model.getGround().expressVectorInAnotherFrame(self.state,
                                                                                         joint.calcReactionOnChildExpressedInGround(
                                                                                             self.state).get(1),
                                                                                         child_body_frame).to_numpy()  # forces
                jointForcesGround[i, :3] = joint.calcReactionOnChildExpressedInGround(self.state).get(0).to_numpy() # moments
                jointForcesGround[i, 3:] = joint.calcReactionOnChildExpressedInGround(self.state).get(1).to_numpy()  # forces

        # Acquire GH-JRF magnitude and relative angle:
        if self.settings['constrainGHjoint']:
            GH_forceVector = np.matmul(constrGH_A, result.x) + JRFnoAct
            GH_forceMagnitude = np.linalg.norm(GH_forceVector)

            cosTheta = np.max((np.min((np.dot(Vec_H2GC, GH_forceVector) / (np.linalg.norm(Vec_H2GC) * np.linalg.norm(GH_forceVector)), 1)), -1))
            GH_forceRelativeAngle = np.rad2deg( np.arccos(cosTheta) )
        else:
            GH_forceRelativeAngle = 0
            GH_forceMagnitude = 0

        # Pack info:
        info = dict(success=result.success,
                    message=result.message,
                    cost=result.fun,
                    tolerance=xDeltaUsed,
                    timeElapsed=timeElapsed.total_seconds(),
                    GHjointMaxAngle=self.jointGHmaxAngle,
                    GHjointAngle=GH_forceRelativeAngle,
                    GHjointForce=GH_forceMagnitude)
        
        # Visualize if enabled:
        if self.settings['visualize']:
            self.model.getVisualizer().show(self.state)
        
        # Convert reserve actuator 'activation' to forces
        activation[self.muscleNum:] = activation[self.muscleNum:] * self.actuatorReserveOptimalForce
        
        return activation, jointForcesChild, jointForcesGround,  info

    # self.model.realizeDynamics(self.state)
    # self.model.getMatterSubsystem()
    # self.model.getMultibodySystem()