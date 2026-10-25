import functions as f
import numpy as np


class Config:

    def __init__(self):
        self.cav_num = 6
        self.vru_num = 20

        self.start_freq = 77 #GHz
        self.totol_bandwidth = 2 #GHz
        self.rb_bandwidth = 1 #GHz
        self.n_actions_h = self.totol_bandwidth // self.rb_bandwidth  # number of available high actions to choose

        self.n_beam_select = 2
        self.cav_power = 10.01  # maximum transmit power of CAV dbm
        self.n_beam_levels = 12
        self.n_power_levels = 5  # number of discrete power levels
        self.n_actions_l = self.n_beam_levels * self.n_power_levels   # number of available low actions to choose

        self.segma_vru = -10 #RCS dbsm
        self.segma_vhl = 20
        self.ratio_r = 10  #超过噪声底噪比例 
        self.ratio_c = 10
        self.miss_aoi = 10 #数据失效AOI 10ms
        self.antena_tgain = 34 #dbi
        self.antena_rgain = 34 #dbi
        self.noise_power = -110 # noise floor without interference dbm
        self.fov = 2 * np.pi / self.n_beam_levels

        self.road_width = 28
        self.road_length = 50
        self.n_links = self.cav_num  # number of simulated direct links in the simulation

        self.slot_interval = 0.02  # interval of one time slot
        self.random_seed = 10  # random seed to control the simulated cellular network              
        self.negative_reward = -1
        self.reward_decay = 0.99
        self.step_per_association = 10
        self.total_slots = 50000   # total time slots in the simulation

        self.lamda = 5
        self.tao = 1
        self.speed = 5
        
        self.cav_search_radius = 100
        

        
        self.velocity = 10 # m/s

