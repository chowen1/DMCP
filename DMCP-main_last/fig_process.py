import json
import matplotlib.pyplot as plt
import numpy as np

def data_porcess():

    window = 500

    filename = 'RatevsTimeslot/nointerlink.json'
    r = []
    with open(filename, 'r') as f:
        data = np.array(json.load(f))
        r.append(data[0])
        for i in range(len(data) -window + 1):
            r.append(np.mean(data[i:i + window]))
    r = np.array(r)
    plt.plot(r, label='rate100', marker="v", markevery=500000, markersize=6, markerfacecolor='none', linewidth=1)

    plt.xlabel('The number of time slots')
    plt.ylabel('Average number of links without interference')
    plt.legend(loc='upper left')
    plt.grid(True)
    plt.show()


data_porcess()
