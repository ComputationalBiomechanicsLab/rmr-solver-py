"""
Author:             FJ van Melis
Created on:         October 1st 2024.
Last updated on:    December 2nd 2024.

PURPOSE:
Functions for use in RMRsolver.py
"""

import numpy as np
import numpy.typing as npt
from typing import Any, List, Dict, Tuple
import opensim

def addTolerance(vector: npt.ArrayLike, tolerance: float) -> Tuple[float,float]:
    """ Returns lower and upper bounds based on vector and tolerance. """

    lowerBound = vector * (1 - tolerance)
    upperBound = vector * (1 + tolerance)

    # switch bounds around for negative entries
    for i in range(len(vector)):
        if abs(vector[i]) < tolerance:
            lowerBound[i] = -tolerance
            upperBound[i] = tolerance
        elif vector[i] < 0:
            temp = lowerBound[i]
            lowerBound[i] = upperBound[i]
            upperBound[i] = temp

    return lowerBound, upperBound



def constructAccelerationConstraint_GH(params: Any, acceleration: npt.ArrayLike, actuatorNum: int, coordinateNum: int) -> Tuple[npt.NDArray]:
    """
    Returns matrices and vector used for constraints:
    1. Constructs A and b of the acceleration constraint in form A*x=b.
    2. Constructs A and b of the GH-JRF constraint in form A*x=b.
    """
    # A = 17x50, x = [a, c] = [33 + 17] = 50x1, b = 17x1
    accelerationNoAct, JRFnoAct = getInducedAcceleration_GH(np.zeros(actuatorNum), params)
    constrAcc_b = acceleration - accelerationNoAct # q_ddot

    actuatorProbe = np.eye(actuatorNum) # 50x50
    constrAcc_A = np.zeros([coordinateNum,actuatorNum]) # 17x50
    constrGH_A = np.zeros([3,actuatorNum])

    for i in range(actuatorNum):
        accelerationSingleActuator, JRFsingleActuator = getInducedAcceleration_GH(actuatorProbe[i,:],params)
        constrAcc_A[:,i] = accelerationSingleActuator - accelerationNoAct
        constrGH_A[:,i] = JRFsingleActuator - JRFnoAct

    return constrAcc_A, constrAcc_b, constrGH_A, JRFnoAct



def constructAccelerationConstraint(params: Any, acceleration: npt.ArrayLike, actuatorNum: int, coordinateNum: int) -> Tuple[npt.NDArray]:
    """
    Returns matrices and vector used for constraints:
    1. Constructs A and b of the acceleration constraint in form A*x=b.
    """
    # A = 17x50, x = [a, c] = [33 + 17] = 50x1, b = 17x1
    accelerationNoAct = getInducedAcceleration(np.zeros(actuatorNum), params)
    constrAcc_b = acceleration - accelerationNoAct # q_ddot

    actuatorProbe = np.eye(actuatorNum) # 50x50
    constrAcc_A = np.zeros([coordinateNum,actuatorNum]) # 17x50

    for i in range(actuatorNum):
        accelerationSingleActuator = getInducedAcceleration(actuatorProbe[i,:],params)
        constrAcc_A[:,i] = accelerationSingleActuator - accelerationNoAct

    return constrAcc_A, constrAcc_b



def GHjointConstraint(x: npt.ArrayLike, params: Any) -> float:
    """
    Returns the status of the conic GH-JRF constraint. Constraint is violated for c > 0.
    """
    vectorGlenoid = params._vectorGlenoid
    maxAngle = params._maxAngle # in degrees
    A = params._A
    JRFnoAct = params._JRFnoAct

    # computing the reaction force vector at the given joint
    vectorJRF = A @ x + JRFnoAct # shapes: (n,m) * (k,) = (n,)
    
    # evaluating the relative angle between the reaction force and Vec_H2GH
    angle = np.dot(vectorGlenoid, vectorJRF) / (np.linalg.norm(vectorGlenoid)*np.linalg.norm(vectorJRF))
    cosTheta = np.maximum(np.minimum(angle, 1), -1)
    relativeAngle = np.real(np.rad2deg(np.arccos(cosTheta))) # in degrees

    # value of the constraint violation (c < 0 is good)
    c = np.square(relativeAngle/maxAngle)-1
    # ceq = 0

    return c



def getInducedAcceleration_GH(activation: npt.ArrayLike, params: Any) -> Tuple[npt.NDArray]:
    """"
    Returns induced accelerations for each coordinate and the GH-JRF for given model, state, muscle activations and reserve actuator controls.
    """

    model = params._model
    state = params._state

    actuators = params._actuators
    actuatorNames = params._actuatorNames

    coordinates = params._coordinates
    coordinateNames = params._coordinateNames
    coordinateNum = params._coordinateNum
    
    muscles = params._muscles
    muscleNames = params._muscleNames
    muscleNum = params._muscleNum

    useMuscles = params._useMuscles
    useControls = params._useControls
    GHjoint = params._GHjoint

    # Initialize the muscles to produce the required forces
    if useMuscles:
        forceActive = params._forceActive
        forcePassive = params._forcePassive
        forceTotal = forceActive * activation[:muscleNum] + forcePassive
        for i, mus in enumerate(muscles):
            mus.setOverrideActuation(state, forceTotal[i])

    model.assemble(state)

    # Initialize the CoordinateActuators to produce the required effect
    if useControls:
        modelControls = params._modelControls
        for i, act in enumerate(actuators):
            if not actuatorNames[i] in muscleNames:
                act.setControls(opensim.Vector(1, activation[i]), modelControls)
        model.realizeVelocity(state)
        model.setControls(state, modelControls)
    else:
        for i, act in enumerate(actuators):
            act.setOverrideActuation(state, activation[i])

    # Realize the model to the acceleration stage
    model.realizeAcceleration(state)

    # Retrieve the simulated accelerations for each coordinate
    inducedAcceleration = np.zeros(coordinateNum)
    for i, coord in enumerate(coordinates):
        inducedAcceleration[i] = coord.getAccelerationValue(state)

    # Get moment and force at the glenohumeral joint
    GHmoment = GHjoint.calcReactionOnParentExpressedInGround(state).get(0).to_numpy()
    GHforce = GHjoint.calcReactionOnParentExpressedInGround(state).get(1).to_numpy()

    return inducedAcceleration, GHforce



def getInducedAcceleration(activation: npt.ArrayLike, params: Any) -> npt.NDArray:
    """"
    Returns induced accelerations for each coordinate for given model, state, muscle activations and reserve actuator controls.
    """

    model = params._model
    state = params._state

    actuators = params._actuators
    actuatorNames = params._actuatorNames

    coordinates = params._coordinates
    coordinateNames = params._coordinateNames
    coordinateNum = params._coordinateNum
    
    muscles = params._muscles
    muscleNames = params._muscleNames
    muscleNum = params._muscleNum

    useMuscles = params._useMuscles
    useControls = params._useControls

    # Initialize the muscles to produce the required forces
    if useMuscles:
        forceActive = params._forceActive
        forcePassive = params._forcePassive
        forceTotal = forceActive * activation[:muscleNum] + forcePassive
        for i, mus in enumerate(muscles):
            mus.setOverrideActuation(state, forceTotal[i])

    model.assemble(state)
    model.realizePosition(state)
    # Initialize the CoordinateActuators to produce the required effect
    if useControls:
        modelControls = params._modelControls
        for i, act in enumerate(actuators):
            if not actuatorNames[i] in muscleNames:
                act.setControls(opensim.Vector(1, activation[i]), modelControls)
        model.realizeVelocity(state)
        model.setControls(state, modelControls)
    else:
        for i, act in enumerate(actuators):
            act.setOverrideActuation(state, activation[i])

    # Realize the model to the acceleration stage
    model.realizeAcceleration(state)

    # Retrieve the simulated accelerations for each coordinate
    inducedAcceleration = np.zeros(coordinateNum)
    for i, coord in enumerate(coordinates):
        inducedAcceleration[i] = coord.getAccelerationValue(state)

    return inducedAcceleration



def get_glenoid_status(model: opensim.Model, state: opensim.State) -> Tuple[float, npt.NDArray]:
    """
    This function returns parameters describing the status of the
    glenohumeral joint in the thoracoscapular model.

    INPUT
    :param model: opensim thoracoscapular model, that must be already provided
                  with markers on the glenoid center, humerus head and glenoid edge
                  (in this order, and they should be the last ones in the markerset)
    :param state: state of the model

    OUTPUT
    :return angle: the maximum angle representing the cone in which the reaction forces must
                   be contained is returned
    :return Vec_H2GC: 3D vector defined between the humeral head center (origin)
                      and the glenoid center. It is expressed in the ground frame
    """

    mkrs = model.getMarkerSet()
    nmkrs = mkrs.getSize()

    # manually hardcode the markers that we want(last 3 in the MarkerSet)
    G_Cent = mkrs.get(nmkrs - 3)
    HH_Cent = mkrs.get(nmkrs - 2)
    G_Edge = mkrs.get(nmkrs - 1)

    # get the location in ground of the 3 markers
    G_Cent_Loc = G_Cent.getLocationInGround(state).to_numpy()
    HH_Cent_Loc = HH_Cent.getLocationInGround(state).to_numpy()
    G_Edge_Loc = G_Edge.getLocationInGround(state).to_numpy()

    # define the vector from the glenoid center to the humerus head
    Vec_H2GC = G_Cent_Loc - HH_Cent_Loc

    # define the vector from the glenoid edge to the humerus head
    Vec_H2GE = G_Edge_Loc - HH_Cent_Loc

    # get the cosine of the angle between the two vectors
    CosTheta = max(min(np.dot(Vec_H2GC, Vec_H2GE) / (np.linalg.norm(Vec_H2GC) * np.linalg.norm(Vec_H2GE)), 1), -1)

    # find the maximum angle to be returned
    angle_rad = np.arccos(CosTheta)
    angle = np.rad2deg(angle_rad)

    # get additional informations about the glenohumeral joint
    Glenoid_Rad = np.linalg.norm(G_Cent_Loc - G_Edge_Loc)
    Head2Glen_dist = np.linalg.norm(HH_Cent_Loc - G_Cent_Loc)

    return angle, Vec_H2GC
    """"
    Returns induced stiffness for the selected coordinates for given model, state, muscle activations and reserve actuator controls.
    """

    model = params._model
    state = params._state

    actuators = params._actuators
    actuatorNames = params._actuatorNames

    coordinates = params._coordinates
    coordinateNames = params._coordinateNames
    
    muscles = params._muscles
    muscleNames = params._muscleNames
    muscleNum = params._muscleNum

    useMuscles = params._useMuscles
    useControls = params._useControls

    # Initialize the muscles to produce the required forces
    if useMuscles:
        forceActive = params._forceActive
        forcePassive = params._forcePassive
        forceTotal = forceActive * activation[:muscleNum] + forcePassive
        for i, mus in enumerate(muscles):
            mus.setOverrideActuation(state, forceTotal[i])

    # Initialize the CoordinateActuators to produce the required effect
    if useControls:
        modelControls = params._modelControls
        for i, act in enumerate(actuators):
            if not actuatorNames[i] in muscleNames:
                act.setControls(opensim.Vector(1, activation[i]), modelControls)
        model.realizeVelocity(state)
        model.setControls(state, modelControls)
    else:
        for i, act in enumerate(actuators):
            act.setOverrideActuation(state, activation[i])

    # Select coordinates to evaluate
    coordinateSelected = []
    for i, coord in enumerate(coordinateNames):
        if coord in coordinate:
            coordinateSelected.append(coordinates[i])

    muscleStiffness = np.zeros(len(actuatorNames))
    for i, mus in enumerate(muscles):
        muscleStiffness[i] = mus.getMuscleStiffness(state)

    # Find moment arms for all muscles around selected coordinates
    momentArms = np.zeros([len(coordinate),len(actuatorNames)])

    solver = opensim.MomentArmSolver(model)
    for i, mus in enumerate(muscles):
        musclePath = mus.getGeometryPath()
        for j, coord in enumerate(coordinateSelected):
            momentArms[j,i] = solver.solve(state, coord, musclePath)

    # Calculate torque induced by all muscles around selected coordinates
    stiffness = momentArms * muscleStiffness

    return stiffness