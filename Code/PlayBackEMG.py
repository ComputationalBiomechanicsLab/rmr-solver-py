"""
Author:             FJ van Melis
Created on:         October 17th 2024.
Last updated on:    October 17th 2024.

PURPOSE:
This class is used to read EMG data from a .exp file.
"""
import time as tm
import numpy as np
import numpy.typing as npt
import os
import opensim
import scipy.signal as ss
import matplotlib.pyplot as plt
from datetime import datetime
from typing import Any, List, Dict, Tuple, Optional, Union

import utilsLoadFile
from PlayBackActive import PlayBackActive

class PlayBackEMG:
    def __init__(self, path: str) -> None:

        # get data from .exp file as dictionary:
        dataRaw = utilsLoadFile.loadEMGfileCSV(path,info=False)

        # assign EMG data to model muscles:
        self.EMGkeys = list(dataRaw.keys())
        self.EMGexp = dict() # for unprocessed EMG
        for key in self.EMGkeys:
            #newKey = EMGmapping[key]
            self.EMGexp.update({key: dataRaw[key]})

        self.EMG = self.EMGexp.copy() # copy for processed EMG
        self.modelKeys = list(self.EMGexp.keys())

    def filter(self, filterType: str='sosfilt', order: int=4, bandType: str='low', cutoff: int=4, ) -> None:
        """ 
        Apply lowpass or highpass filter to EMG with specified cut-off frequency in Hz.
        Use before cropping with setTime()!!!
        """
        assert filterType == 'sosfilt' or filterType == 'lfilter' or filterType == 'filtfilt'
        assert bandType == 'low' or bandType == 'high'

        if filterType == 'sosfilt':
            filterSos= ss.butter(order, cutoff, btype=bandType, fs=self.freqExp, output='sos')
            for key in self.modelKeys:
                newData = ss.sosfilt(filterSos, self.EMG[key], axis=0)
                self.EMG.update({key: newData})
        else:
            filterB, filterA = ss.butter(order, cutoff, btype=bandType, fs=self.freqExp, output='ba')
            if filterType == 'lfilter':
                for key in self.modelKeys:
                    newData = ss.lfilter(filterB, filterA, self.EMG[key], axis=0)
                    self.EMG.update({key: newData})
            else:
                for key in self.modelKeys:
                    newData = ss.filtfilt(filterB, filterA, self.EMG[key], axis=0)
                    self.EMG.update({key: newData})


    
    def rect(self) -> None:
        """
        Rectify EMG data.
        """
        for key in self.modelKeys:
            newData = abs(self.EMG[key])
            self.EMG.update({key: newData})



    def setTime(self, start: Optional[int]=None, stop: Optional[int]=None, hz: int=100) -> None:
        """ Set time window and frequency and crop. """

        if not start == None:
            start = np.argmin( abs(np.array(self.timeExp) - start) )
        
        if not stop == None:
            stop = np.argmin( abs(np.array(self.timeExp) - stop) ) + 1

        if hz <= self.freq:
            interval = int( np.floor(self.freqExp/hz) )
            self.freq = 1/ (1/self.freqExp * interval)
            print(f">>> EMG play rate set to {self.freq}Hz")
        else:
            print(f">>> Requested play rate exceeds data rate. Play rate is remains {self.freq}Hz")

        self.setIterator(start,stop,interval)



    def setIterator(self, start: Optional[int]=None, stop: Optional[int]=None, interval: int=1) -> None:
        """ Set iteration window and interval and crop. """

        self.time = self.timeExp[start:stop:interval]
        for key in self.modelKeys:
            croppedEMG = self.EMG[key][start:stop:interval]
            self.EMG.update({key: croppedEMG})


    
    def sync(self,motion= PlayBackActive) -> None:
        """ Sync EMG with motion instant. """

        hz = motion.getFrequency()
        steps = motion.getSteps()
        timeStart = motion.getTimeStart()


        interval = int( np.floor(self.freqExp/hz) )
        start = np.argmin( np.abs( np.array(self.timeExp) - timeStart ) )
        stop = start + steps * interval
        self.setIterator(start,stop,interval)

    def all(self) -> Tuple[npt.NDArray,Dict]:
        """ Return time and EMG for all muscles at all time instants. """

        return self.EMG

    def getSteps(self) -> int:
        steps = len(self.time)

        return steps

    def getMuscleNames(self) -> list:
        muscleNames = self.modelKeys

        return muscleNames
