import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import os
import subprocess
from threading import Thread

# 打印速度项

data_path = '../log/'

if True and os.path.exists(data_path + 'veldata.txt'):
    veldata = pd.read_csv(data_path + 'veldata.txt', sep=' ').values
    start_time = veldata[0, 0]
    veldata[:, 0] -= start_time

    # for i in range(0, len(veldata)):
    #     if veldata[i, 1] > 3 or veldata[i, 4] > 3:
    #         veldata[i, 1] = 3
    #         veldata[i, 4] = 3

    plt.figure(figsize=(8, 6))
    plt.plot(veldata[:, 0], veldata[:, 1], label='GPS Velocity', color="tab:blue")
    plt.plot(veldata[:, 0], veldata[:, 2], label='Fusion Velocity', color="tab:red")

    # if os.path.exists(data_path + 'imudata.txt'):
    #     imudata = pd.read_csv(data_path + 'imudata.txt', sep=' ').values
    #     imudata[:, 0] -= start_time
    #     for i in range(0, len(imudata)):
    #         imudata[i, 1] -= imudata[i, 4]

    #     print(imudata)
    #     plt.plot(imudata[:, 0], imudata[:, 1], label='IMU a_x', color="tab:orange")


    plt.title(data_path)
    plt.legend()
    plt.grid()
    plt.tight_layout()

plt.show()