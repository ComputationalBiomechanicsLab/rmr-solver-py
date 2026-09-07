"""
Author:             FJ van Melis, Sabrina Hörmann
Created on:         September 19th 2024.
Last updated on:    April 10th 2024.

PURPOSE:
Analyse a motion and plot the results.

USAGE:
See runShoulderModel.py for an example on how to use this class!

__init__:
    Set up motion analysis by providing motion, solver and EMG data (optional).
runAll():
    Use the provided solver to solve for activations for the whole motion and return the solution.
addPlot():
    Specify a plot to be displayed when plotAll() is called.
plotAll():
    Plot specified figures (configured with addPlot()) for one or more solutions.
compareAll():
    Compare two or more solutions to each other.
compareEMG():
    Compare one or more solutions to EMG (if provided)
"""
import numpy as np
import numpy.typing as npt
from typing import Any, List, Dict, Tuple, Optional, Union
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

from PlayBackActive import PlayBackActive
from RMRsolver import RMRsolver
from PlayBackEMG import PlayBackEMG

class MotionAnalysis:
    def __init__(self, motion: PlayBackActive, solver: RMRsolver, EMGplayer: Optional[PlayBackEMG] = None) -> None:
        """ 
        Construct MotionAnalysis class used for solving the muscle redundacy problem for a recorded motion. 
        Provide with a classes PlayBackActive, RMRsolver and PlayBackEMG (optional).
        """
        self.motion = motion
        self.solver = solver
        if EMGplayer is None:
            self.hasEMG = False
        else:
            self.EMGplayer = EMGplayer
            self.hasEMG = True

        # extract motion and model parameters
        self.info = []
        self.steps = motion.getSteps()
        self.coordinateNum = solver.coordinateNum
        self.muscleNum = solver.muscleNum
        self.actuatorNum = solver.actuatorNum
        self.jointNum = solver.jointNum

        self.coordinateNames = solver.coordinateNames
        self.actuatorNames = solver.actuatorNames
        self.jointNames = solver.settings['outputJoint']
        self.coordinateCartesians = motion.getCartesians()

        self.plotKwargs = []



    def addPlot(self, plotType: str, plotRange: list=[]) -> None:
        """ 
        Add plot to analysis.

        :Parameters:

        plotType: ``str`` |
            Type of plot to add
                1. ``'activation'`` for muscle activations over time
                2. ``'coordinate'`` for model coordinates over time 
                3. ``'info'`` for solver info over time
                4. ``'GHjoint'`` for glenohumeral joint reaction force angle and magnitude over time
        plotRange: ``List`` |
            list of actuator or coordinate names to plot (default plots all available)
        """
        self.plotKwargs.append(dict(plotType= plotType,
                                    plotRange= plotRange))
        


    def runAll(self, countInTerminal: bool=True) -> Dict:
        """ 
        Run RMR_solver solver on data provided by the PlayBackActive class and return the results in a dictionary.

        :Parameters:
        countInTerminal: ``bool`` (default = True) |
            Toggle time instant counter in terminal.

        :Returns:
        ``Dict``
            Arrays for time, position, speed, acceleration, activation and info
        """

        # Retrieve all (recorded) input data:
        time, position, speed, acceleration = self.motion.all()

        if self.hasEMG:
            self.EMGdict = self.EMGplayer.all()

            # Convert EMG dictionary to EMG array with zeros for non-EMG data
            self.EMG = np.zeros([self.steps,self.actuatorNum])

            for i, name in enumerate(self.actuatorNames):
                if name in self.EMGdict:
                    EMGarray = self.EMGdict[name]
                    if len(EMGarray) >= self.steps:
                        self.EMG[:,i] = EMGarray[:self.steps]
                    else:
                        print(i)
                        self.EMG[:,i] = EMGarray[:]

        # Prepare arrays & dictionaries
        activation = np.zeros([self.steps, self.actuatorNum])
        jointForcesChild = np.zeros([self.steps, self.jointNum, 6])
        jointForcesGround = np.zeros([self.steps, self.jointNum, 6])
        passiveMuscleForces = np.zeros([self.steps, self.muscleNum])

        solverInfo = dict(success=[],
                          message=[],
                          cost=[],
                          tolerance=[],
                          timeElapsed=[],
                          GHjointMaxAngle=[],
                          GHjointAngle=[],
                          GHjointForce=[])
        
        print(f">>> INFO: Running RMR_solver solver on {self.steps} time instants. This may take a while...")
        timeRMRstart = datetime.now()

        # Apply RMR_solver solver to all time instances:
        for i in range(self.steps):
            if countInTerminal:
                print(i)

            if self.hasEMG:
                if not self.jointNum == 0:
                    activation[i,:], jointForcesChild[i,:], jointForcesGround[i,:], passiveMuscleForces[i,:], info = self.solver.solve(time[i],position[i,:],speed[i,:],acceleration[i,:],self.EMG[i,:])
                else:
                    activation[i,:], jointForcesChild[:], jointForcesGround[i,:], passiveMuscleForces[i,:], info = self.solver.solve(time[i],position[i,:],speed[i,:],acceleration[i,:],self.EMG[i,:])

            else:
                if not self.jointNum == 0:
                    activation[i,:], jointForcesChild[i,:], jointForcesGround[i,:], passiveMuscleForces[i,:], info = self.solver.solve(time[i],position[i,:],speed[i,:],acceleration[i,:])
                else:
                    activation[i,:], jointForcesChild[i,:], jointForcesGround[i,:], passiveMuscleForces[i,:], info = self.solver.solve(time[i],position[i,:],speed[i,:],acceleration[i,:])

            for key in list(solverInfo.keys()):
                solverInfo[key].append(info[key])

        timeRMRelapsed = datetime.now() - timeRMRstart
        print(f">>> INFO: Solver finished evaluation of {self.steps} time instants in {timeRMRelapsed} ({self.steps/timeRMRelapsed.total_seconds():.2f}Hz).")

        # Pack results:
        results = dict(time= time,
                       position= position,
                       speed= speed,
                       acceleration= acceleration,
                       activation= activation,
                       jointForcesChild=jointForcesChild,
                       jointForcesGround=jointForcesGround,
                       passiveMuscleForces=passiveMuscleForces,
                       info = solverInfo)

        return results

    def plotAll(self, results: List[Dict], labels: List=[]) -> None:
        """
        Plot all requested figures for one or more sets of results.

        :Parameters:
        results: ``List`` of ``Dict``|
            List of results (output from runAll()) to plot.
        labels: ``List`` of ``str`` |
            Labels for the corresponding results.
        """
        # create labels if empty
        if len(labels) == 0:
            for i in range(len(results)):
                labels.append('Result ' + str(i))

        # unpack results and list them per data type
        time, position, speed, acceleration, activation, jointForcesChild, jointForcesGround, passiveMuscleForces, solverInfo = unpackResults(results)
        
        # plot requested plots
        for figIndex, kwargs in enumerate(self.plotKwargs):
            fig = plt.figure(figIndex+1,figsize=(13,7),dpi=144,layout='constrained') # made to fit 15.6" 16:9 (13.6"x7.65") 1080p screen

            if kwargs['plotType'] == 'activation':
                # read coordinates to plot
                plotRange = kwargs['plotRange']
                if len(plotRange) == 0:
                    indices = np.arange(self.muscleNum)
                else:
                    indices = []
                    for muscle in plotRange:
                        indices.append( self.actuatorNames.index(muscle) )
                    
                N = len(indices)

                # set up grid of plots and convert to list of axes
                numCols = int(np.ceil(np.sqrt(N)))
                numRows = int(np.ceil((N / numCols)))
                ax = fig.subplots(numRows,numCols,sharey='row',sharex='col')
                ax = np.reshape(ax,numRows*numCols)
                for axes in ax:
                    axes.tick_params(labelsize='x-small')

                # fill grid of plots with results
                for resIndex, res in enumerate(results):
                    resultLabel = labels[resIndex]
                    for plotIndex, muscleIndex in enumerate(indices):
                        if not plotIndex == 0:
                            resultLabel = '_' + resultLabel
                        ax[plotIndex].plot(time[resIndex],activation[resIndex][:,muscleIndex],label= resultLabel)
                        ax[plotIndex].set_ylim(0,1)
                        ax[plotIndex].set_title(self.actuatorNames[muscleIndex], fontsize= 'x-small')
                
                # add EMG for comparison
                if self.hasEMG:
                    EMGkeys = list(self.EMGdict.keys())
                    resultLabel = 'EMG'

                    EMGindices = []
                    for muscle in EMGkeys:
                        EMGindices.append( self.actuatorNames.index(muscle) )

                    for plotIndex, muscleIndex in enumerate(indices):
                        if muscleIndex in EMGindices:
                            muscle = self.actuatorNames[muscleIndex]
                            ax[plotIndex].plot(self.EMGtime,self.EMGdict[muscle],label= resultLabel)
                            resultLabel = '_' + resultLabel

                # shape back list of axes and add legend
                ax = np.reshape(ax,(numRows,numCols))
                fig.legend(loc='outside right upper',fontsize='x-small')

            if kwargs['plotType'] == 'reserve':
                N = self.actuatorNum - self.muscleNum

                # set up grid of plots and convert to list of axes
                numCols = int(np.ceil(np.sqrt(N)))
                numRows = int(np.ceil((self.actuatorNum - self.muscleNum) / numCols))
                ax = fig.subplots(numRows,numCols,sharex='col')
                ax = np.reshape(ax,numRows*numCols)
                for axes in ax:
                    axes.tick_params(labelsize='x-small')

                # fill grid of plots with results
                for resIndex, res in enumerate(results):
                    resultLabel = labels[resIndex]
                    for i in range(N):
                        if not i == 0:
                            resultLabel = '_'
                        reserveForce = activation[resIndex][:,i+self.muscleNum]

                        ax[i].set_title(self.actuatorNames[i+self.muscleNum], fontsize= 'x-small')
                        ax[i].plot(time[resIndex],reserveForce,label= resultLabel)

                        reserveForceMax = np.max(abs(reserveForce))
                        if resIndex == 0:
                            if reserveForceMax > 1.0:
                                ax[i].set_ylim(-reserveForceMax,reserveForceMax)
                            else:
                                ax[i].set_ylim(-1,1)
                        
                # shape back list of axes and add legend
                ax = np.reshape(ax,(numRows,numCols))
                fig.legend(loc='outside right upper',fontsize='x-small')

            if kwargs['plotType'] == 'coordinate':
                
                # read coordinates to plot
                plotRange = kwargs['plotRange']
                if len(plotRange) == 0:
                    indices = np.arange(self.coordinateNum)
                else:
                    indices = []
                    for coord in plotRange:
                        indices.append( self.coordinateNames.index(coord) )
                N = len(indices)

                # set up grid of plots and convert to list of axes
                numCols = N
                numRows = 3
                ax = fig.subplots(numRows,numCols,sharey='row',sharex='col')
                ax = np.reshape(ax,numRows*numCols)
                for axes in ax:
                    axes.tick_params(labelsize='x-small')

                # fill grid of plots with results
                for resIndex, res in enumerate(results):
                    resultLabel = labels[resIndex]
                    for plotIndex, coordIndex in enumerate(indices):
                        if not plotIndex == 0:
                            resultLabel = '_' + resultLabel
                        else:
                            ax[plotIndex].set_ylabel("[deg] or [m]")
                            ax[plotIndex+N].set_ylabel("[deg/s] or [m/s]")
                            ax[plotIndex+N*2].set_ylabel("deg/s^2] or [m/s^2]")

                        coordName = self.coordinateNames[coordIndex]
                        if coordName in self.coordinateCartesians:
                            pos = position[resIndex][:,coordIndex]
                            spd = speed[resIndex][:,coordIndex]
                            acc = acceleration[resIndex][:,coordIndex]
                        else:
                            pos = np.rad2deg(position[resIndex][:,coordIndex])
                            spd = np.rad2deg(speed[resIndex][:,coordIndex])
                            acc = np.rad2deg(acceleration[resIndex][:,coordIndex])

                        ax[plotIndex].plot(time[resIndex], pos, label= resultLabel)
                        ax[plotIndex+N].plot(time[resIndex], spd)
                        ax[plotIndex+N*2].plot(time[resIndex], acc)

                        ax[plotIndex].set_title(self.coordinateNames[coordIndex], fontsize= 'x-small')
                        ax[plotIndex+N*2].set_xlabel("Time [s]")

                # shape back list of axes and add legend
                ax = np.reshape(ax,(numRows,numCols))
                fig.legend(loc='outside right upper',fontsize='x-small')

            if kwargs['plotType'] == 'info':

                ax1 = fig.add_subplot(311)
                ax2 = fig.add_subplot(312)
                ax3 = fig.add_subplot(313)

                ax1.tick_params(labelsize='small')
                ax2.tick_params(labelsize='small')
                ax3.tick_params(labelsize='small')
                
                ax1.set_ylabel('Success')
                ax1.set_title('Solver results')

                ax2.set_ylabel('Cost')

                ax3.set_xlabel('Time [s]')
                ax3.set_ylabel('Solve time [s]')
                        
                for resIndex, res in enumerate(results):
                    resultLabel = labels[resIndex]
                    ax1.plot(time[resIndex],solverInfo[resIndex]['success'],label= resultLabel)
                    ax2.plot(time[resIndex],solverInfo[resIndex]['cost'])
                    ax3.plot(time[resIndex],solverInfo[resIndex]['timeElapsed'])

                fig.legend(loc='outside right upper',fontsize='x-small')

            if kwargs['plotType'] == 'GHjoint':
                maxAngle = solverInfo[0]['GHjointMaxAngle'][0]

                ax1 = fig.add_subplot(121)
                ax2 = fig.add_subplot(122)

                for resIndex, res in enumerate(results):
                    resultLabel = labels[resIndex]
                    ax1.plot(time[resIndex],solverInfo[resIndex]['GHjointAngle'],label=resultLabel)
                    ax2.plot(time[resIndex],solverInfo[resIndex]['GHjointForce'])

                ax1.plot([time[0][0], time[0][-1]], [maxAngle, maxAngle], label='Maximum angle') # draw constraint limit angle
                ax1.set(xlabel='Time [s]',ylabel='Angle [deg]',title='GH-JRF angle and magnitude')

                ax2.set(xlabel='Time [s]',ylabel='Magnitude [N]')
            
                fig.legend(loc='outside right upper')
        
            if kwargs['plotType'] == 'joint':
                # Read joints to plot:
                plotRange = kwargs['plotRange']
                if len(plotRange) == 0:
                    indices = np.arange(self.jointNum)
                else:
                    indices = []
                    for joint in plotRange:
                        indices.append( self.jointNames.index(joint) )
                    
                N = len(indices)*6

                # Set up grid of plots and convert to list of axes
                numCols = int(6)
                numRows = int(len(indices))
                ax = fig.subplots(numRows,numCols,sharex='col')
                ax = np.reshape(ax,numRows*numCols)
                for axes in ax:
                    axes.tick_params(labelsize='x-small')
                
                directionLabel = ['Mx','My','Mz','Fx','Fy','Fz']
                # Fill grid of plots with results
                for resIndex, res in enumerate(results):
                    resultLabel = labels[resIndex]
                    for jointCount, jointIndex in enumerate(indices):
                        for direction in range(6):
                            plotIndex = jointCount*6 + direction
                            if not plotIndex == 0:
                                resultLabel = '_' + resultLabel
                            ax[plotIndex].plot(time[resIndex],jointForcesChild[resIndex][:,jointIndex,direction],label= resultLabel)
                            ax[plotIndex].set_title(self.jointNames[jointIndex] + ' ' + directionLabel[direction], fontsize= 'x-small')
                # Shape back list of axes and add legend
                ax = np.reshape(ax,(numRows,numCols))
                fig.legend(loc='outside right upper',fontsize='x-small')
        plt.show()

    def saveToCSV(self, outputForce, outputMuscleActivations, outputReserveForce, outputPassiveMuscleForces, results: List[Dict], labels: List = []) -> None:
        # unpack results and list them per data type
        time, position, speed, acceleration, activation, jointForcesChild, jointForcesGround, passiveMuscleForces, solverInfo = unpackResults(results)
        directionLabel = ['Mx', 'My', 'Mz', 'Fx', 'Fy', 'Fz']
        indices = [0]
        df = pd.DataFrame()
        df['time']= time[0]
        # Fill grid of plots with results
        for resIndex, res in enumerate(results):
            for jointCount, jointIndex in enumerate(indices):
                for direction in range(6):
                    df[f'{directionLabel[direction]}_child'] = jointForcesChild[resIndex][:, jointIndex, direction]
                for direction in range(6):
                    df[f'{directionLabel[direction]}_ground'] = jointForcesGround[resIndex][:, jointIndex, direction]
        df.to_csv(path_or_buf=outputForce, index=False)
        df_activation = pd.DataFrame()
        df_activation_muscle = pd.DataFrame()
        indices = np.arange(self.muscleNum)
        df_activation_muscle['time'] = time[0]
        for resIndex, res in enumerate(results):
            for plotIndex, muscleIndex in enumerate(indices):
                df_activation_muscle[f'{self.actuatorNames[muscleIndex]}'] = activation[resIndex][:, muscleIndex]
            df_activation = pd.concat([df_activation, df_activation_muscle])
        df_activation.to_csv(path_or_buf=outputMuscleActivations, index=False)

        N = self.actuatorNum - self.muscleNum

        df_reserve = pd.DataFrame()
        df_reserve['time'] = time[0]
        # fill grid of plots with results
        for resIndex, res in enumerate(results):
            for i in range(N):
                df_reserve[f'{self.actuatorNames[i + self.muscleNum]}']  = activation[resIndex][:, i + self.muscleNum]
        df_reserve.to_csv(path_or_buf=outputReserveForce, index=False)

        df_passive_muscle = pd.DataFrame()
        df_passive_muscle_single = pd.DataFrame()
        for resIndex, res in enumerate(results):
            for plotIndex, muscleIndex in enumerate(indices):
                df_passive_muscle_single[f'{self.actuatorNames[muscleIndex]}'] = passiveMuscleForces[resIndex][:,
                                                                                 muscleIndex]
            df_passive_muscle = pd.concat([df_passive_muscle, df_passive_muscle_single])
        df_passive_muscle.to_csv(path_or_buf=outputPassiveMuscleForces, index=False)



    def compareAll(self, results: List[Dict], labels: List=[]) -> None:
        """
        Compares the first results to all following results and reports differences to the terminal.

        :Parameters:
        results: ``List`` of ``Dict``|
            List of results (output from runAll()) to plot.
        labels: ``List`` of ``str`` |
            Labels for the corresponding results.
        """
        # check if there are at least two results provided
        if len(results) <= 1:
            raise AssertionError("Please provide at least two sets of results for comparison.")
        
        # generate labels if none are provided
        if len(labels) == 0:
            for i in range(len(results)):
                labels.append('Result ' + str(i))
        
        # unpack results and list them per data type
        time, position, speed, acceleration, activation, jointForcesChild, jointForcesGround, passiveMuscleForces, solverInfo = unpackResults(results)
        
        print('>>>INFO: SOLVER COMPARISON REPORT\n')

        for resIndex in range(1, len(results)):
            if not self.steps == len(time[resIndex]):
                print(f">>> WARNING: Results labeled {labels[resIndex]} cannot be compared to {labels[0]}, because they contain a different number of time instants ({len(time[resIndex])} and {len(time[0])}, respectively).")
            else:
                print(f'*** Comparison {labels[resIndex]} to {labels[0]} ***')

                # time MAE
                timeErrorStart = abs( time[0][0] - time[resIndex][0] )
                timeErrorEnd = abs( time[0][-1] - time[resIndex][-1] )
                timeErrorCum = np.sum( time[0] - time[resIndex] )
                print(f'Start time error: {timeErrorStart} [s]')
                print(f'End time error: {timeErrorEnd} [s]')
                print(f'Time line cumulative error: {timeErrorCum} [s]')

                # movement MAE (mean absolute error)
                positionMAE = []
                speedMAE = []
                accelerationMAE = []

                fig = plt.figure(resIndex*10+1,figsize=(13,7),dpi=144,layout='constrained')
                ax1 = fig.add_subplot(311)
                ax2 = fig.add_subplot(312)
                ax3 = fig.add_subplot(313)

                for coordIndex in range(self.coordinateNum):
                    positionMAE.append(np.mean( abs( position[0][:,coordIndex] - position[resIndex][:,coordIndex] ))) 
                    speedMAE.append( np.mean( abs( speed[0][:,coordIndex] - speed[resIndex][:,coordIndex] )))
                    accelerationMAE.append( np.mean( abs( acceleration[0][:,coordIndex] - acceleration[resIndex][:,coordIndex] )))

                ax1.barh(self.coordinateNames,positionMAE)
                ax2.barh(self.coordinateNames,speedMAE)
                ax3.barh(self.coordinateNames,accelerationMAE)

                ax1.set_title('Position MAE [rad]',fontsize='small')
                ax1.tick_params(labelsize='x-small')

                ax2.set_title('Speed MAE [rad/s]',fontsize='small')
                ax2.tick_params(labelsize='x-small')

                ax3.set_title('Acceleration MAE [rad/s^2]',fontsize='small')
                ax3.tick_params(labelsize='x-small')

                print(f"Position MAE [rad]: {np.mean(positionMAE):.5f} mean (max {np.max(positionMAE):.5f} for {self.coordinateNames[ np.argmax(positionMAE) ]})")
                print(f"Speed MAE [rad/s]: {np.mean(speedMAE):.5f} mean (max {np.max(speedMAE):.5f} for {self.coordinateNames[ np.argmax(speedMAE) ]})")
                print(f"Acceleration MAE [rad/s^2]: {np.mean(accelerationMAE):.5f} mean (max {np.max(accelerationMAE):.5f} for {self.coordinateNames[ np.argmax(accelerationMAE) ]})")

                # cost MAE
                costMAE = np.mean(abs( np.array(solverInfo[0]['cost']) - np.array(solverInfo[resIndex]['cost']) ))
                print(f"Solution cost MAE: {costMAE:.3f}")

                # muscle activation MAE
                actuatorMAE = []

                fig = plt.figure(resIndex*10+2,figsize=(13,7),dpi=144,layout='constrained')
                ax1 = fig.add_subplot(121)
                ax2 = fig.add_subplot(122)

                ax1.set_title('Muscle activation MAE',fontsize='small')
                ax1.tick_params(labelsize='x-small')

                ax2.set_title('Reserve actuator force MAE [N]',fontsize='small')
                ax2.tick_params(labelsize='x-small')

                for actIndex in range(self.actuatorNum):
                    actuatorMAE.append(np.mean( abs( activation[0][:,actIndex] - activation[resIndex][:,actIndex] )))

                muscleMAE = actuatorMAE[:self.muscleNum]
                reserveMAE = actuatorMAE[self.muscleNum:]

                ax1.barh(self.actuatorNames[:self.muscleNum],muscleMAE)
                ax2.barh(self.actuatorNames[self.muscleNum:],reserveMAE)

                print(f"Muscle activation MAE: {np.mean(muscleMAE):.3f} mean (max {np.max(muscleMAE):.3f} for {self.actuatorNames[ np.argmax(muscleMAE) ]})")
                print(f"Reserve actuator activation MAE: {np.mean(reserveMAE):.3f} mean (max {np.max(reserveMAE):.3f} for {self.actuatorNames[ self.muscleNum + np.argmax(reserveMAE) ]})")

        plt.show()



    def compareEMG(self,results: List[Dict],  labels: List=[], muscles: List=[]) -> None:
        """
        Compares the results to EMG.

        :Parameters:
        results: ``List`` of ``Dict``|
            List of results (output from runAll()) to plot.
        labels: ``List`` of ``str`` |
            Labels for the corresponding results.
        muscles: ``List`` of ``str`` |
            List of muscles to include in the comperison.
        """
        # check if there are at least two results provided
        if not self.hasEMG:
            raise AssertionError("Please provide EMG data for comparison.")
        
        # unpack results and list them per data type
        time, position, speed, acceleration, activation, jointForcesChild, jointForcesGround, passiveMuscleForces, solverInfo = unpackResults(results)
        
        # create labels if empty
        if len(labels) == 0:
            for i in range(len(results)):
                labels.append('Result ' + str(i))

        print('>>> INFO: EMG COMPARISON REPORT \n')
        muscleMAEdict = dict()

        for resIndex in range(len(results)):
            if not len(activation[resIndex]) == len(self.EMG):
                print(f">>> WARNING: {labels[resIndex]} cannot be compared to EMG, because they contain a different number of time instants ({len(time[resIndex])} and {len(self.EMGtime)}")
            else:
                print(f'*** Comparison to {labels[resIndex]} ***')

                # cost MAE
                costMean = np.mean( np.array(solverInfo[resIndex]['cost']) ) 
                print(f"Solution cost mean: {costMean:.3f}")

                # muscle activation MAE
                muscleMAE = []
                if len(muscles) == 0:
                    EMGkeys = list(self.EMGdict.keys())
                else:
                    EMGkeys = muscles

                for i, muscle in enumerate(EMGkeys):
                    muscleIndex = self.actuatorNames.index(muscle)
                    muscleMAE.append(np.mean( abs( activation[resIndex][:,muscleIndex] - self.EMG[:,muscleIndex] )))
                
                muscleMAEmean = np.mean(muscleMAE)
                muscleMAE.append(muscleMAEmean)
                muscleMAEdict.update({labels[resIndex]: muscleMAE})
        
                print(f"Muscle activation MAE: {muscleMAEmean:.3f} mean (max {np.max(muscleMAE):.3f} for {EMGkeys[ np.argmax(muscleMAE) ]})")

        # plotting:
        fig = plt.figure(1,figsize=(13,7),dpi=144,layout='constrained')
        ax = plt.subplot()

        EMGkeys.append('Mean')
        pos = np.arange(len(EMGkeys))  # the label locations
        width = 0.25  # the width of the bars
        multiplier = 0

        for result, MAE in muscleMAEdict.items():
            offset = width * multiplier
            rects = ax.barh(pos + offset, MAE, width, label=result)
            ax.bar_label(rects, padding=3,fontsize='x-small')
            multiplier += 1

        ax.set_title('Muscle activation MAE compared to EMG',fontsize='small')
        ax.tick_params(labelsize='x-small')
        ax.set_yticks(pos + width/2, EMGkeys)
        ax.legend(loc='upper right',ncols=len(labels),fontsize='small')
     
        plt.show()



    def runLive(self, sleep: bool=False):
        pass



def unpackResults(results: List[Dict]) -> Tuple[List,List,List,List,List,List,List,List]:
    """
    Unpacks multiple results and returns lists for each data type in the following order:

    *time, postition, speed, acceleration, activation, solverInfo*
    """
    time = []
    position = []
    speed = []
    acceleration = []
    activation = []
    jointForcesChild = []
    jointForcesGround = []
    passiveMuscleForces = []
    solverInfo = []

    for i, res in enumerate(results):
        time.append(res['time'])
        position.append(res['position'])
        speed.append(res['speed'])
        acceleration.append(res['acceleration'])
        activation.append(res['activation'])
        jointForcesChild.append(res['jointForcesChild'])
        jointForcesGround.append(res['jointForcesGround'])
        passiveMuscleForces.append(res['passiveMuscleForces'])
        solverInfo.append(res['info'])

    return time, position, speed, acceleration, activation, jointForcesChild, jointForcesGround, passiveMuscleForces, solverInfo