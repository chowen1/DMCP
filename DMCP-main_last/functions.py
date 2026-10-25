import scipy.io as sio
import numpy as np
import math
from config import Config


def get_beambook():
    mdict = sio.loadmat('beambook/beambook_dft44.mat')
    beam_book = mdict['ans']
    return beam_book

def dB2num(dB):
    num = 10 ** (dB / 10)
    return num

def dBm2num(dB):
    num = 10 ** (dB / 10)
    return num * 0.001

def num2dB(num):
    dB = 10 * np.log10(num)
    return dB

def get_distance(src_location, tar_location):
    temp = tar_location - src_location
    d = np.linalg.norm(temp)
    return d

def get_azimuth(src_location, tar_location):
    temp = tar_location - src_location
    d = np.linalg.norm(temp)
    azimuth = np.arccos(temp[0] / d)
    if temp[1] < 0:
        azimuth = 2 * np.pi - azimuth
    return azimuth


def sigmoid(d):
    k = 0.5
    y = 1 / (1 + math.exp(k * (d / Config().d_msb - 1)))
       
    return y

