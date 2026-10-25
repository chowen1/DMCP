from config import Config
import functions as f
import numpy as np
import math

class Channel:

    def __init__(self, src, tar):

        self.src = src
        self.tar = tar
        self.index = np.array([src.index, tar.index])
        self._cal_aoda_()
        self.segma_vru = f.dB2num(Config().segma_vru)
        self.segma_vhl = f.dB2num(Config().segma_vhl)
        self.ratio_c = Config().ratio_c
        self.ratio_r = Config().ratio_r
        self.d = f.get_distance(self.src.location, self.tar.location)

        self.tgain = f.dB2num(Config().antena_tgain)
        self.rgain = f.dB2num(Config().antena_rgain)
        # for saving outdated csi
        self.power, self.freq = None, None
        self.beam_aod, self.beam_aoa = None, None
        self.singletrip_power, self.singletrip_power10, self.singletrip_power11, self.singletrip_power20, self.singletrip_power21 = None, None, None, None, None
        self.roundtrip_power, self.roundtrip_power10, self.roundtrip_power11, self.roundtrip_power20, self.roundtrip_power21 = None, None, None, None, None


        self.utility, self.utility10, self.utility11, self.utility20, self.utility21 = 0, 0, 0, 0, 0
        self.mos, self.mos10, self.mos11, self.mos20, self.mos21 = 0, 0, 0, 0, 0
        self.SINR_r, self.SINR_r10, self.SINR_r11, self.SINR_r20, self.SINR_r21 = 0, 0, 0, 0, 0
        self.SINR_c, self.SINR_c10, self.SINR_c11, self.SINR_c20, self.SINR_c21 = 0, 0, 0, 0, 0
        self.IN_r, self.IN_r10, self.IN_r11, self.IN_r20, self.IN_r21 = 0, 0, 0, 0, 0
        self.IN_c, self.IN_c10, self.IN_c11, self.IN_c20, self.IN_c21 = 0, 0, 0, 0, 0

        self.vru_eff_rlink, self.vhl_eff_rlink, self.vhl_eff_clink = False, False, False #默认均为false
        self.interlink_list_r, self.interlink_list_c = [], []
        self.interferers_r, self.interferers_c = 0, 0
        
        
        self._check_is_link_()
        
        
    def _cal_aoda_(self): #目标的到达角 aoa、aod均为绝对角度，以正东方向（x正半轴）为0度，不随车辆运动姿态变化
        self.aod = f.get_azimuth(self.src.location, self.tar.location)
        if self.aod > np.pi:
            self.aoa = self.aod - np.pi
        else:
            self.aoa = self.aod + np.pi
       

    def _check_is_link_(self): #判断是否落在波束宽度内
        self._cal_aoda_()
        self.d = f.get_distance(self.src.location, self.tar.location)
        beam_index = self.projection(self.aod)
        if  beam_index in self.src.beam_index:
            self.is_link = True
            self.power = self.src.power[self.src.beam_index.index(beam_index)]
            self.freq = self.src.freq
            self.beam_aod = self.src.beambook[beam_index]
            if self.tar.index < 100: #为vhl链路
                beam_index1 = self.projection(self.aoa)
                self.beam_aoa = self.tar.beambook[beam_index1]
            self._get_singletrip_power_()
            self._get_roundtrip_power_()
        else:
            self.is_link = False
            self.power, self.freq = None, None
            self.beam_aod, self.beam_aoa = None, None #在车辆第几边
            self.singletrip_power, self.roundtrip_power = None, None

    def projection(self, angle):   #返回index
        if (Config().n_beam_levels * 2 - 1) / (Config().n_beam_levels * 2) * 2 * np.pi < angle or angle < 1 / (Config().n_beam_levels * 2) * 2 * np.pi:
            return 0
        else:
            return int((angle - 1 / (Config().n_beam_levels * 2) * 2 * np.pi) // (1 / Config().n_beam_levels * 2 * np.pi) + 1)


    def _get_singletrip_power_(self): 
        self.singletrip_power = self.power * self.tgain * self.rgain * (3e8 / (self.freq * 1e9)) ** 2 / (4 * np.pi * self.d) ** 2
        
    def _get_roundtrip_power_(self):
        if self.tar.index > 100: #vru
            self.roundtrip_power = self.power * self.tgain * self.rgain * (3e8 / (self.freq * 1e9)) ** 2 * self.segma_vru / ((4 * np.pi) ** 3 * (self.d) ** 4)
        else: #vhl
            self.roundtrip_power = self.power * self.tgain * self.rgain * (3e8 / (self.freq * 1e9)) ** 2 * self.segma_vhl / ((4 * np.pi) ** 3 * (self.d) ** 4)
            
    def evaluate_mos(self, utility):
        if utility:
            self.mos = Config().lamda * math.log10(Config().tao * utility)
        else:
            self.mos = 0

    def _save_csi_(self, ir_change):

        if ir_change:

            self.h2 = self.h1
            self.h1 = self.h

            self.r_power21 = self.r_power11
            self.r_power11 = self.r_power

            self.gain21 = self.gain11
            self.gain11 = self.gain

            if self.is_link:
                self.IN21 = self.IN11
                self.IN11 = self.IN

                self.SINR21 = self.SINR11
                self.SINR11 = self.SINR

                self.utility11 = self.utility

                self.mos21 = self.mos11
                self.mos11 = self.mos

                self.interferer_neighbors21 = self.interferer_neighbors11
                self.interferer_neighbors11 = self.interferer_neighbors

                self.interfered_neighbors21 = self.interfered_neighbors11
                self.interfered_neighbors11 = self.interfered_neighbors
        else:

            self.r_power20 = self.r_power10
            self.r_power10 = self.r_power

            self.gain20 = self.gain10
            self.gain10 = self.gain

            if self.is_link:
                self.IN20 = self.IN10
                self.IN10 = self.IN

                self.SINR20 = self.SINR10
                self.SINR10 = self.SINR

                self.utility10 = self.utility

                self.mos20 = self.mos10
                self.mos10 = self.mos

                self.interferer_neighbors20 = self.interferer_neighbors10
                self.interferer_neighbors10 = self.interferer_neighbors

                self.interfered_neighbors20 = self.interfered_neighbors10
                self.interfered_neighbors10 = self.interfered_neighbors

    def update(self, ir_change):

        self._save_csi_(ir_change)

        self._cal_csi_(ir_change)
