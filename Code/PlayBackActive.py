"""
Author:             FJ van Melis
Created on:         September 17th 2024.
Last updated on:    October 10th 2024.

PURPOSE:
This class is used to read data series from a .mot file, simulating live data updates.

USAGE:
See runVisualizationPlayBack.py.

"""
import time as tm
import numpy as np
import opensim
import scipy.signal as ss
import matplotlib.pyplot as plt
from datetime import datetime
from typing import List, Optional

import utilsLoadFile as utilsLoad

class PlayBackActive:
    def __init__(self, model: opensim.Model, path: str, name: str, cartesians: List=['ground_thorax_tx','ground_thorax_ty','ground_thorax_tz']) -> None:

        # get coordinate names in order from the model:
        coordinateSet = model.getCoordinateSet()
        self.coordinateNum = coordinateSet.getSize()
        self.cartesians = cartesians

        self.coordinateNames = []
        for i in range(self.coordinateNum):
            self.coordinateNames.append(coordinateSet.get(i).getName())

        # get data from .mot file as dictionary:
        dataRaw = utilsLoad.loadMotFile(path, name)

        # extract time and (average) frequency:
        self.timeMot = np.array(dataRaw['time'])
        self.freqMot = (len(self.timeMot) - 1) / (self.timeMot[-1] - self.timeMot[0]) # frequency of .mot file
        self.freq = self.freqMot # frequency of the output arrays (changes when cropped)

        # extract keys:
        del dataRaw['time']
        self.keys = list(dataRaw.keys())

        # structure coordinate data in array in order:
        self.positionRad = np.zeros([len(self.timeMot),self.coordinateNum])
        self.positionDeg = np.zeros(np.shape(self.positionRad))
        self.speedRad = np.zeros(np.shape(self.positionRad))
        self.speedDeg = np.zeros(np.shape(self.positionRad))
        self.accelerationRad = np.zeros(np.shape(self.positionRad))
        self.accelerationDeg = np.zeros(np.shape(self.positionRad))

        for i, name in enumerate(self.coordinateNames):
            self.positionDeg[:,i] = dataRaw[name]
            
            if name in cartesians:
                self.positionRad[:,i] = dataRaw[name]
            else:
                self.positionRad[:,i] = np.deg2rad(dataRaw[name])

        print(1/self.freq)
        # get time derivatives of position:
        for i in range(self.coordinateNum):
            self.speedRad[:,i] = np.gradient(self.positionRad[:,i], 1/self.freqMot, axis=0)
            self.speedDeg[:,i] = np.gradient(self.positionDeg[:,i], 1/self.freqMot, axis=0)

            self.accelerationRad[:,i] = np.gradient(self.speedRad[:,i], 1/self.freqMot, axis=0)
            self.accelerationDeg[:,i] = np.gradient(self.speedDeg[:,i], 1/self.freqMot, axis=0)

        # initialize iterator sleep counter:
        self.i = 0 # iterator
        self.timeMotionLast = 0.
        self.timeCodeLast = 0.

        # initial output data:
        self.setIterator()



    def filter(self, hz: int=6) -> None:
        """ 
        Apply lowpass filter to position with specified cut-off frequency in Hz.
        Speed and acceleration are recomputed accordingly.
        """
        filterB, filterA = ss.butter(4, hz, btype='low', fs=self.freqMot, output='ba')
        self.positionRad = ss.filtfilt(filterB, filterA, self.positionRad, axis=0, padtype='odd')
        self.positionDeg = ss.filtfilt(filterB, filterA, self.positionDeg, axis=0, padtype='odd')

        # get new time derivatives of position:
        for i in range(self.coordinateNum):
            self.speedRad[:,i] = np.gradient(self.positionRad[:,i], 1/self.freqMot, axis=0)
            self.speedDeg[:,i] = np.gradient(self.positionDeg[:,i], 1/self.freqMot, axis=0)

            self.accelerationRad[:,i] = np.gradient(self.speedRad[:,i], 1/self.freqMot, axis=0)
            self.accelerationDeg[:,i] = np.gradient(self.speedDeg[:,i], 1/self.freqMot, axis=0)

        # update output data:
        self.setIterator()



    def setTime(self, start: Optional[int]=None, stop: Optional[int]=None, hz: int=5) -> None:
        """ Set time window and frequency and crop. """

        if not start == None:
            start = np.argmin( abs(np.array(self.timeMot) - start) )
        
        if not stop == None:
            stop = np.argmin( abs(np.array(self.timeMot) - stop) )

        if hz <= self.freqMot:
            interval = int( np.floor(self.freqMot/hz) )
            self.freq = 1/ (1/self.freqMot * interval)
            print(f">>> INFO: Motion play rate set to {self.freq}Hz")
        else:
            print(f">>> WARNING: Requested play rate exceeds data rate. Play rate is remains {self.freq}Hz")

        self.setIterator(start,stop,interval)



    def setIterator(self, start: Optional[int]=None, stop: Optional[int]=None, interval: int=1) -> None:
        """ Set iteration window and interval and crop. """

        self.time = self.timeMot[start:stop:interval]
        self.position = self.positionRad[start:stop:interval]
        self.speed = self.speedRad[start:stop:interval]
        self.acceleration = self.accelerationRad[start:stop:interval,:]



    def all(self) -> tuple:
        """ Return time, postition, speed and acceleration at all time instants. """

        return self.time, self.position, self.speed, self.acceleration



    def next(self, sleep: bool=True) -> tuple:
        """ Return time, position, speed and acceleration at next time instant. """

        # extract data line:
        time = self.time[self.i]
        position = self.position[self.i,:].tolist()
        speed = self.speed[self.i,:].tolist()
        acceleration = self.acceleration[self.i,:].tolist()

        # sleep to simulate data delay:
        if sleep:
            timeToPass = time - self.timeMotionLast # the time that should pass between current time stamp and last time stamp
            timePassed = datetime.now() - self.timeCodeLast # the actual time passed due to processing

            timeSleep = timeToPass - timePassed
            if timeSleep > 0:
                tm.sleep(timeSleep)

        # update:
        self.timeMotionLast = time
        self.timeCodeLast = datetime.now()
        self.i += 1

        # reset when end of file is reached:
        if self.i >= len(self.time):
            self.i = 0

        return time, position, speed, acceleration
    


    def getSteps(self) -> int:
        steps = len(self.time)
        return steps

    def getFrequency(self) -> float:

        return self.freq
    
    def getTimeStart(self) -> float:
        return self.time[0]
    
    def getCartesians(self) -> List:
        return self.cartesians
    
    def plot(self, coordinate: list=['all'], angleUnit: str='rad', compare: bool=True) -> None:
        """ 
        Plot position, speed and acceleration of the motion. 
        Is affected by cropping and filtering.
        Only suitable for angular coordinates.
        Can compare original motion to cropped & filtered motion.
        """
        if coordinate == ['all']:
            coordinate = self.coordinateNames
            
        plotTime = self.timeMot

        if angleUnit == 'deg':
            plotPosition = self.positionDeg
            plotSpeed = self.speedDeg
            plotAcceleration = self.accelerationDeg
            labels = ['deg','deg/s','deg/s^2']
            compare = False
        else:
            plotPosition = self.positionRad
            plotSpeed = self.speedRad
            plotAcceleration = self.accelerationRad
            labels = ['rad','rad/s','rad/s^2']

        fig = plt.figure(figsize=(13,7),dpi=144,layout='constrained')

        N = len(coordinate)
        for i, coord in enumerate(coordinate):
            coordIndex = self.coordinateNames.index(coord)

            ax1 = fig.add_subplot(3,N,1+i)
            ax1.plot(plotTime,plotPosition[:,coordIndex])
            ax1.set_title(self.keys[coordIndex],fontsize='small')
            ax1.tick_params(labelsize='x-small')

            ax2 = fig.add_subplot(3,N,N+1+i)
            ax2.plot(plotTime,plotSpeed[:,coordIndex])
            ax2.tick_params(labelsize='x-small')

            ax3 = fig.add_subplot(3,N,2*N+1+i)
            ax3.plot(plotTime,plotAcceleration[:,coordIndex])
            ax3.set_xlabel('Time [s]',fontsize='small')
            ax3.tick_params(labelsize='x-small')

            if compare:
                ax1.plot(self.time,self.position[:,coordIndex])
                ax2.plot(self.time,self.speed[:,coordIndex])
                ax3.plot(self.time,self.acceleration[:,coordIndex])
                
            if i == 0:
                ax1.set_ylabel(labels[0],fontsize='small')
                ax2.set_ylabel(labels[1],fontsize='small')
                ax3.set_ylabel(labels[2],fontsize='small')
        
        if compare:
            fig.legend(labels=['Original','Filtered'],loc='outside right upper',fontsize='x-small')

        plt.show()