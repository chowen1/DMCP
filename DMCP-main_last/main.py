from environment import WirelessNetwork as WN
import json
import random
import numpy as np
from config import Config
import os
import scipy.io as sio
import warnings
import copy
warnings.filterwarnings('ignore')
os.environ["CUDA_VISIBLE_DEVICES"] = '1'

c = Config()
random.seed(c.random_seed)
np.random.seed(c.random_seed)
WN = WN()
ave_success_rate = []
ave_SINR = []
ave_INR = []
ave_interlink = []
ave_efflink = []
ave_nointerlink = []
r_cav_last = 0
test = []
r_accumulate = np.zeros(Config().cav_num)
s_obs = WN.observe()
step_per_association = Config().step_per_association
show_reward = []
count_beam_action = []
count_power_action = []
count_aoi = []
for _ in range(c.total_slots):
    print(_)

    s_obs = WN.observe()
    actions = WN.choose_actions(s_obs)
    WN.update(ir_change=False, flag='dsc', actions=actions)

    ave_success_rate.append(WN.get_ave_success_rate())
    ave_SINR.append(WN.get_ave_sinr())
    ave_INR.append(WN.get_ave_inr())
    ave_interlink.append(WN.get_ave_interlink())
    ave_efflink.append(WN.get_ave_efflink())
    ave_nointerlink.append(WN.get_ave_nointerlink())
    
    r_cav = WN.give_rewards()
    show_reward.append(r_cav)
    r_cav_new = r_cav - r_cav_last

    s_obs_next = WN.observe()
    WN.save_transitions(s_obs, actions, r_cav_new, s_obs_next)
    r_cav_last = r_cav

    count_beam_temp = []
    count_power_temp = []
    count_aoi_temp = []
    for cav in WN.cav_list:
        count_beam_temp.append(copy.copy(cav.beam))
        count_power_temp.append(copy.copy(cav.power))
        count_aoi_temp.append(cav.aoi)
    count_beam_action.append(count_beam_temp)
    count_power_action.append(count_power_temp)
    count_aoi.append(count_aoi_temp)

    if _ > 256:
        WN.train_dqns()
WN.save_models()

filename = 'RatevsTimeslot/rate.json'
with open(filename, 'w') as f:
    json.dump(ave_success_rate, f)

filename = 'RatevsTimeslot/sinr.json'
with open(filename, 'w') as f:
    json.dump(ave_SINR, f)

filename = 'RatevsTimeslot/inr.json'
with open(filename, 'w') as f:
    json.dump(ave_INR, f)

filename = 'RatevsTimeslot/inter.json'
with open(filename, 'w') as f:
    json.dump(ave_interlink, f)

filename = 'RatevsTimeslot/efflink.json'
with open(filename, 'w') as f:
    json.dump(ave_efflink, f)

filename = 'RatevsTimeslot/nointerlink.json'
with open(filename, 'w') as f:
    json.dump(ave_nointerlink, f)

reward_m = np.array(show_reward)
sio.savemat('QoS/show_reward.mat', {'show_reward': reward_m})


beam_m = np.array(count_beam_action)
sio.savemat('QoS/proposedBeam.mat', {'proposedBeam': beam_m})

power_m = np.array(count_power_action)
sio.savemat('QoS/proposedPow.mat', {'proposedPow': power_m})

aoi_m = np.array(count_aoi)
sio.savemat('QoS/proposedAoi.mat', {'proposedAoi': aoi_m})




