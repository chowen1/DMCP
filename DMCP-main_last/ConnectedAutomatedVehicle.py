
import numpy as np
import random
import functions as f
from config import Config
from DRQN import drqn
import copy


class ConnectedAutomatedVehicle:

    def __init__(self, location, index):

        self.location = location
        self.index = index
        self.flag = 'CAV'
        self.nn = drqn()
        self.max_power = f.dBm2num(Config().cav_power)
        self.start_freq = Config().start_freq
        self.rb = Config().rb_bandwidth
        self.beambook = np.arange(Config().n_beam_levels) * np.pi *2 / (Config().n_beam_levels) 
        self.powerbook = np.arange(Config().n_power_levels + 1) * self.max_power / (Config().n_power_levels)
        self.powerbook = np.delete(self.powerbook, 0)
        self.freqbook = np.arange(Config().n_actions_h) * self.rb +  self.start_freq
        self.inter_power, self.inter_noise_power = [], []
        self.old_aoi = []
        self.success_detection = 0
        self.aoi = 0
        self.IN_r, self.IN_c, self.IN_r11, self.IN_c11, self.IN_r12, self.IN_c12 = 0, 0, 0, 0, 0, 0
        self.SINR_r, self.SINR_c, self.SINR_r11, self.SINR_c11, self.SINR_r12, self.SINR_c12 = 0, 0, 0, 0, 0, 0
        self.n_link, self.n_link11, self.n_link12, = 0, 0, 0
        self.n_eff_link, self.n_eff_link11, self.n_eff_link12, = 0, 0, 0
        self.n_nointer_link, self.n_nointer_link11, self.n_nointer_link12, = 0, 0, 0

        self._init_book_select_()
        self._init_beacon_()
        self._init_params_()
    
    def _init_book_select_(self):
        self.beam_index, self.beam = [], []
        self.freq_index, self.freq = [], []
        self.power_index, self.power = [], []
        for i in range(Config().n_beam_select):
            self.power_index.append(random.randint(0, Config().n_power_levels-1))
            self.power.append(self.powerbook[self.power_index[i]])
        for i in range(Config().n_beam_select):
            self.beam_index.append(random.randint(0, Config().n_beam_levels-1))
            self.beam.append(self.beambook[self.beam_index[i]])
        
        self.freq_index = 0
        self.freq = self.freqbook[self.freq_index]
        

    def _init_beacon_(self):
        self.neighbortable, self.neighbortable1 = {}, {}
        self.statetable = []
        self.statetable.append(self.index)
        self.statetable.append(self.location)
        self.statetable.append(self.freq)
        self.neighbortable[np.random.randint(0 + 100, Config().vru_num + 100)] = [np.array([np.random.randint(0, Config().road_length), np.random.randint(0, Config().road_width)]), 0, float("inf"), float("inf")]
        i = list(range(Config().cav_num))
        i.remove(self.index)
        self.neighbortable[i[np.random.randint(0, Config().cav_num - 1)]] = [np.array([np.random.randint(0, Config().road_length), random.choice([12, 16]),]), self.freqbook[random.randint(0, Config().n_actions_h-1)], float("inf"), float("inf")]


    def _init_params_(self):

        self.beam_index1, self.beam_index2 = None, None
        self.power_index1, self.power_index2 = None, None
        self.freq_index1, self.freq_index2 = None, None
        self.power1, self.power2 = None, None
        self.beam1, self.beam2 = None, None
        self.freq1, self.freq2 = None, None
        
    def _save_params_(self):

        self.beam_index2 = copy.copy(self.beam_index1)
        self.beam_index1 = copy.copy(self.beam_index)

        self.power_index2 = copy.copy(self.power_index1)
        self.power_index1 = copy.copy(self.power_index)

        self.power2 = copy.copy(self.power1)
        self.power1 = copy.copy(self.power)

        self.beam2 = copy.copy(self.beam1)
        self.beam1 = copy.copy(self.beam)

    def update_state(self):
        self.statetable[1] = copy.copy(self.location)
        self.statetable[2] = self.freq

    def update_aoi(self):
        for key in self.neighbortable.keys():
            self.neighbortable[key][-1] += 1
            self.neighbortable[key][-2] += 1
            self.neighbortable1 = copy.deepcopy(self.neighbortable) #neighbortable1 为广播信息，避免靠前cav的neighbortable改变

    def _save_csi_(self):
        self.IN_r12 = self.IN_r11
        self.IN_r11 = self.IN_r
        self.IN_c12 = self.IN_c11
        self.IN_c11 = self.IN_c
        self.SINR_r12 = self.SINR_r11
        self.SINR_r11 = self.SINR_r 
        self.SINR_c12 = self.SINR_c11
        self.SINR_c11 = self.SINR_c
        self.n_link12 = self.n_link11
        self.n_link11 = self.n_link
        self.n_eff_link12 = self.n_eff_link11
        self.n_eff_link11 = self.n_eff_link
        self.n_nointer_link12 = self.n_nointer_link11
        self.n_nointer_link11 = self.n_nointer_link

    def discover(self, links=None): #通过链路获得信息
        self.old_aoi = [] #记录更新时原有aoi情况
        
        for link in links:
            if link.vru_eff_rlink == True and link.src.index == self.index:
                self._update_table_(link.tar)
            else:
                if link.vhl_eff_rlink == True and link.src.index == self.index:
                    self._update_table_(link.tar)
                if link.vhl_eff_clink == True and link.tar.index == self.index:   #合并列表
                    self._merge_table_(link.src)

    def _update_table_(self, obj=None):
        if obj.index not in self.neighbortable.keys():
            self.old_aoi.append(float("inf"))
            self.neighbortable[obj.index] = copy.deepcopy([obj.location, 0, 0, float("inf")])   #key为目标id,val依次为目标坐标、指示该目标没有频率(vru)、感知信息aoi（可由通信或感知链路更新）、通信信息时间戳（只能由通信链路更新）
        else:
            self.old_aoi.append(self.neighbortable[obj.index][-2] - 0)
            self.neighbortable[obj.index][0] = copy.deepcopy(obj.location) #更新位置
            self.neighbortable[obj.index][-2] = 0 #更新感知参数aoi

    def _merge_table_(self, obj=None):
        #先获得对方statetable
        if obj.statetable[0] not in self.neighbortable.keys():
            self.old_aoi.append(float("inf")) #感知aoi
            self.old_aoi.append(float("inf")) #通信aoi
        else:
            self.old_aoi.append(self.neighbortable[obj.statetable[0]][-2] - 0)
            self.old_aoi.append(self.neighbortable[obj.statetable[0]][-1] - 0) 
        self.neighbortable[obj.statetable[0]] = copy.deepcopy(obj.statetable[-2:] + [0,0]) #直接获得
        #其次获得对方neighbortable
        for key in obj.neighbortable1.keys():
            if key not in self.neighbortable.keys() and key != self.index:
                self.old_aoi.append(float("inf")) #感知aoi
                self.old_aoi.append(float("inf")) #通信aoi
                self.neighbortable[key] = copy.deepcopy(obj.neighbortable1[key])
            elif key != self.index:
                if obj.neighbortable1[key][-2] < self.neighbortable[key][-2]: #更新位置
                    self.old_aoi.append(self.neighbortable[key][-2] - obj.neighbortable1[key][-2])
                    self.neighbortable[key][0] = copy.deepcopy(obj.neighbortable1[key][0])
                    self.neighbortable[key][-2] = obj.neighbortable1[key][-2]
                if obj.neighbortable1[key][-1] < self.neighbortable[key][-1]: #更新频率
                    self.old_aoi.append(self.neighbortable[key][-1] - obj.neighbortable1[key][-1])
                    self.neighbortable[key][1] = obj.neighbortable1[key][1]
                    self.neighbortable[key][-1] = obj.neighbortable1[key][-1]

    def get_success_p(self):  #成功检测率
        a = b = 0
        for key in self.neighbortable.keys():
            if self.neighbortable[key][-2] < Config().miss_aoi:
                a += 1
                b += self.neighbortable[key][-2]
            
        self.success_detection = a / (Config().cav_num + Config().vru_num - 1)
        if a != 0:
            self.aoi = b / a

        

    def take_action(self, action=None, weight=None):
        self._save_params_()
        for i in range(len(action)):
            if action[i] is not None:
                self.power_index[i] = action[i] % Config().n_power_levels
                self.beam_index[i] = action[i] // Config().n_power_levels
                self.beam[i] = self.beambook[self.beam_index[i]]
                self.power[i] = self.powerbook[self.power_index[i]]
            if weight is not None:
                if np.linalg.norm(weight) != 0:
                    self.beam = weight / np.linalg.norm(weight)
                    self.power = np.square(np.linalg.norm(weight))
                else:
                    self.beam = np.zeros(Config().n_antennas, dtype=np.complex)
                    self.power = 0



