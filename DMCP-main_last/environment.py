""" simulator for cellular networks """

from ConnectedAutomatedVehicle import ConnectedAutomatedVehicle as CAV
from VulnerableRoadUser  import VulnerableRoadUser  as VRU
from config import Config
from channel import Channel
import functions as f
import numpy as np
import random
import copy

class WirelessNetwork:

    def __init__(self):
        """ initialize the cellular network """
        self.config = Config()
        self._generate_cav_()
        self._generate_vru_()
        self._establish_channels_()


    def _generate_cav_(self):
        self.cav_list = []
        for ind in range(Config().cav_num):
            self.cav_list.append(CAV(np.array([np.random.randint(0, Config().road_length), random.choice([12, 16])]), ind))

    def _generate_vru_(self):
        self.vru_list = []
        for ind in range(Config().vru_num):
            self.vru_list.append(VRU(np.array([np.random.randint(0, Config().road_length), np.random.randint(0, Config().road_width)]), 100+ind,
                                [Config().road_length, Config().road_width]))

    def _establish_channels_(self):
        self.channels = []
        for cav in self.cav_list:
            for vru in self.vru_list:
                self.channels.append(Channel(cav, vru))
            a = copy.copy(self.cav_list)
            a.remove(cav)
            for cav1 in a:
                self.channels.append(Channel(cav, cav1))
        self._get_links_()
        self._get_eff_links_()
        self._evaluate_cav_performance_()

    def _update_check_link_(self):
        for channel in self.channels:
            channel._check_is_link_()
        self._get_links_()
        self._get_eff_links_()

    def _get_links_(self): #vhl_link已经有干扰，但不一定有效检测、通信
        self.links = []
        for channel in self.channels:
            if channel.is_link:   
                self.links.append(channel)
        self._evaluate_link_performance_()

    
    def _evaluate_link_performance_(self):
        """ evaluate the performance of the link """
        for link in self.links:
            self._get_interlink_list_(link)
            self._get_link_interferers_(link)
            link.IN_r = link.interferers_r + f.dBm2num(Config().noise_power)
            link.SINR_r = link.roundtrip_power / link.IN_r
            if link.tar.index < 100:
                link.IN_c = link.interferers_c + f.dBm2num(Config().noise_power)
                link.SINR_c = link.singletrip_power / link.IN_c
    
    def _get_interlink_list_(self, selected_link):
        """ Search for channels that meet the given conditions """
        selected_link.interlink_list_r= []
        selected_link.interlink_list_c= []  #清零
        if selected_link.tar.index > 100: #vru_link
            a = copy.copy(self.links)
            a.remove(selected_link)
            for link in a:
                if link.tar.index == selected_link.src.index and link.freq == selected_link.freq and link.beam_aoa == selected_link.beam_aod:
                    selected_link.interlink_list_r.append(link)
        else: #vhl_link
            a = copy.copy(self.links)
            a.remove(selected_link)
            for link in a:
                if link.tar.index == selected_link.src.index and link.freq == selected_link.freq and link.beam_aoa == selected_link.beam_aod:
                    selected_link.interlink_list_r.append(link)
                else:
                    if link.tar.index == selected_link.tar.index and link.freq == selected_link.freq and link.beam_aoa == selected_link.beam_aoa: 
                        selected_link.interlink_list_c.append(link)
                    elif link.src.index == selected_link.tar.index and link.freq == selected_link.freq and link.beam_aod == selected_link.beam_aoa:
                        selected_link.interlink_list_c.append(link)

    def _get_link_interferers_(self, selected_link):
        """ get the set of all the interferers """
        selected_link.interferers_r, selected_link.interferers_c = 0, 0
        for link in selected_link.interlink_list_r:
            selected_link.interferers_r += link.singletrip_power
        if selected_link.tar.index < 100: #vhl_link 额外通信干扰
            for link in selected_link.interlink_list_c:
                if link.tar.index == selected_link.tar.index:
                    selected_link.interferers_c += link.singletrip_power
                else:
                    selected_link.interferers_c += link.roundtrip_power
        

    def _get_eff_links_(self): #获得有效检测、通信link
        for link in self.links:
            if link.tar.index >= 100 and link.roundtrip_power > link.IN_r * Config().ratio_r:
                link.vru_eff_rlink = True
            elif link.tar.index < 100:
                if link.roundtrip_power > link.IN_r * Config().ratio_r:
                    link.vhl_eff_rlink = True
                if link.singletrip_power > link.IN_c * Config().ratio_c:
                    link.vhl_eff_clink = True
                

    def _evaluate_cav_performance_(self):
        """ evaluate the performance of the vru """
        for cav in self.cav_list:
            cav._save_csi_()
            cav.n_link = cav.n_eff_link = cav.n_nointer_link = 0
            IN_r, IN_c, SINR_r, SINR_c = [], [], [], []
            for link in self.links:
                if link.index[0] == cav.index:
                    cav.n_link += 1                  
                    IN_r.append(link.IN_r) 
                    SINR_r.append(link.SINR_r)
                    if link.IN_r <= f.dBm2num(Config().noise_power):
                        cav.n_nointer_link += 1
                    if link.SINR_r >= Config().ratio_r:
                        cav.n_eff_link += 1
                    if link.index[1] < 100: #vhl_link才有通信干扰
                        cav.n_link += 1
                        IN_c.append(link.IN_c)
                        SINR_c.append(link.SINR_c)
                        if link.IN_c <= f.dBm2num(Config().noise_power):
                            cav.n_nointer_link += 1
                        if link.SINR_c >= Config().ratio_c:
                            cav.n_eff_link += 1
            if len(IN_r) == 0:
                cav.IN_r = cav.IN_c = 0  #表示无效
                cav.SINR_r = cav.SINR_c = 0
            elif len(IN_c) == 0:
                cav.IN_c = 0  
                cav.SINR_c = 0
                cav.IN_r = sum(IN_r) / len(IN_r)
                cav.SINR_r = sum(SINR_r) / len(SINR_r)
            else:  
                cav.IN_r = sum(IN_r) / len(IN_r)
                cav.IN_c = sum(IN_c) / len(IN_c)
                cav.SINR_r = sum(SINR_r) / len(SINR_r)
                cav.SINR_c = sum(SINR_c) / len(SINR_c) 

    def _update_discovery_(self):
        for cav in self.cav_list:
            cav.update_aoi()
        for cav in self.cav_list:
            cav.discover(self.links)
            cav.get_success_p() 
      
    def update(self, ir_change, flag=None, actions=None, weights=None, ):
        """ update the cellular network status due to channel fading or beamformers update"""
        if ir_change:
            for channel in self.channels:
                channel.update(ir_change)
             
        else:
            if actions is not None:
                self._take_actions_(actions=actions)
                self._update_check_link_()
                self._evaluate_cav_performance_()
                if flag is not None:
                    self._update_discovery_()
                
                    
            if weights is not None:
                self._take_actions_(weights=weights)

    def random_choose_actions(self):
        """ random take actions"""
        actions = []
        for _ in range(self.config.n_links):
            actions.append(random.randint(0, self.config.n_actions - 1))
        return np.array(actions)

    def _take_actions_(self, actions=None, weights=None):
        """ CAVs take the given actions"""
        if actions is not None:
            for index in range(actions.shape[0]):
                self.cav_list[index].take_action(action=actions[index])
        if weights is not None:
            for index in range(weights.shape[1]):
                self.cav_list[index].take_action(weight=weights[:, index])

    def _reset_(self):
        for _ in range(Config().cav_num):
            actions = self.random_choose_actions()
            self.update(ir_change=False, actions=actions)
            self.update(ir_change=True)


    def observe(self): #从neighbortable获取上次检测目标aod与其他频率车辆aod
        """ obtain the states of the cavs"""
        observations = []
        for cav in self.cav_list:
            target_info, target_info_2 = [], []
            temp = dict(sorted(cav.neighbortable.items(), key=lambda x: x[1][2], reverse=False))#按照位置AOI升序
            if len(temp.keys()) >= 20:
                for key in list(temp.keys())[:20]:
                    target_info.append(self.channels[0].projection(f.get_azimuth(cav.location, temp[key][0])))
            else:
                for key in temp.keys():
                    target_info.append(self.channels[0].projection(f.get_azimuth(cav.location, temp[key][0])))
                target_info += [target_info[np.random.randint(0, len(target_info))] for i in range(20  - len(target_info))]
            temp_2 = dict(sorted(cav.neighbortable.items(), key=lambda x: x[1][-1], reverse=False))#按照频率AOI升序
            for key in temp_2.keys():
                if key < 100: #只保留车辆目标，观测频率与aod
                    if temp_2[key][1] == 0:
                        target_info_2.append(-1)
                    else:
                        target_info_2.append(cav.freqbook.tolist().index(temp_2[key][1]))
                    target_info_2.append(self.channels[0].projection(f.get_azimuth(cav.location, temp_2[key][0])))
            if len(target_info_2) / 2 < (Config().cav_num - 1): #未知频率重复补全 
                a=[]
                for i in range((Config().cav_num - 1)  - int(len(target_info_2) / 2)):
                    c = np.random.randint(0, len(target_info_2) / 2) * 2
                    a.append(target_info_2[c])
                    a.append(target_info_2[c+1])
                target_info_2 += a
            local_information = target_info + target_info_2 + cav.power_index + cav.beam_index + [cav.freq_index, cav.IN_r / f.dBm2num(Config().noise_power), cav.IN_c / f.dBm2num(Config().noise_power)]
            observations.append(local_information)

        return np.array(observations)

    @staticmethod
    def dist(cav, vru):
        return np.sqrt(np.power(cav.location[0] - vru.location[0], 2) +
                       np.power(cav.location[1] - vru.location[1], 2))

    def adjacent(self):
        adj = []
        for cav1 in self.cav_list:
            sur = []
            for cav in self.cav_list:
                dis = self.dist(cav1, cav)
                if 0 < dis <= cav1.search:
                    sur.append(cav.index)
            adj.append(sur)
        return adj

    def give_rewards(self):
        """ calculated the rewards of all the CAVs, conditioned on the association"""
        rewards = []
        for cav in self.cav_list:
            reward = 0
            for value in cav.neighbortable.values():
                if value[-2] < Config().miss_aoi:
                    reward += Config().miss_aoi - value[-2] 
            rewards.append(reward)
        return np.array(rewards)

    def count_vru_qos(self):
        vru_qos = []
        for vru in self.vru_list:
            vru_qos.append(vru.QoS)
        return vru_qos

    def save_transitions(self, s, a, r, s_):
        for cav in self.cav_list:
            i = cav.index
            cav.nn.save_transition(s[i, :], a[i][0], r[i], s_[i, :])

    def train_dqns(self):
        """ train the DRQN of each cav"""
        for cav in self.cav_list:
            cav.nn.learn()

    def choose_actions(self, s):
        """ choose actions """
        actions = []
        for cav in self.cav_list:
            actions.append(cav.nn.choose_action(s[cav.index, :]))
        return np.array(actions)

    def save_models(self):
        i = 1
        """ save models """
        for cav in self.cav_list:
            cav.nn.save_model('low_ap_{}'.format(i))
            i += 1

    def load_models(self):
        i = 1
        for cav in self.cav_list:
            cav.nn.load_mod(i)
            i += 1

    def get_ave_success_rate(self):  #平均成功检测率
        s = 0
        for cav in self.cav_list:
            s += cav.success_detection
        return s/Config().cav_num

    def get_ave_sinr(self):  #平均SINR
        s = 0
        for cav in self.cav_list:
            s += (cav.SINR_r + cav.SINR_c) / 2
        return s/Config().cav_num

    def get_ave_inr(self):  #平均INR
        s = 0
        for cav in self.cav_list:
            s += (cav.IN_r + cav.IN_c) / 2 / f.dBm2num(Config().noise_power)
        return s/Config().cav_num

    def get_ave_interlink(self):  #平均干扰率
        s = 0
        n = Config().cav_num
        for cav in self.cav_list:
            if cav.n_link != 0:
                s += 1 - cav.n_nointer_link / cav.n_link
            else: 
                n -= 1
        if n != 0:
            return s/n
        else:
            return 1
    
    def get_ave_efflink(self):  #平均efflink
        s = 0
        for cav in self.cav_list:
            s += cav.n_eff_link
        return s/Config().cav_num

    def get_ave_nointerlink(self):  #平均nointerlink
        s = 0
        for cav in self.cav_list:
            s += cav.n_nointer_link
        return s/Config().cav_num


    def get_all_rates(self):
        rates = []
        for link in self.links:
            rates.append(link.utility)
        return rates

