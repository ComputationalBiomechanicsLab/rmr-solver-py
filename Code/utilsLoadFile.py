"""
Author:             FJ van Melis, Sabrina Hörmann
Created on:         September 13th 2024.
Last updated on:    April 8th 2026.

PURPOSE:
Several functions for opening files:
    - opensim model
    - .mot motion file
    - .csv EMG file
"""

import tkinter
from tkinter import filedialog
import pandas as pd

root = tkinter.Tk()
import numpy as np


def loadMotFile(path: str, name: str, info: bool=False) -> dict:
    """ Returns <name>.mot in target <folder>. """
    file = open(path)

    # find end of file header
    while True:
        line = file.readline()
        if not line.find("endheader") == -1:
            break
    
    # get column names
    line = file.readline()
    dataHeader = line.split()

    # get data
    dataList = file.readlines()
    dataArray = np.zeros([len(dataList),len(dataHeader)])
    data = dict()

    for i, line in enumerate(dataList):
        dataArray[i,:] = line.split()
    
    # structure data in dictionary
    for i, head in enumerate(dataHeader):
        col = dataArray[:,i].tolist()
        data[head] = col

    # print info
    if info:
        print(f"\n--- INFO OF {name}.mot ---\n")
        print(f"FILE SIZE:\n{len(dataHeader)} columns and {len(dataList)} rows")
        print("\nDATA COLUMNS:")
        for head in dataHeader:
            print(head)

        print(f"\nTIME RANGE:\n{data['time'][0]} to {data['time'][-1]}\n")

    file.close()

    return data

def loadEMGfileCSV(path: str, info: bool = True) -> dict:
    """ Loads <name>.csv file. """

    df = pd.read_csv(path)
    data = {col: df[col].to_numpy() for col in df.columns}

    return data