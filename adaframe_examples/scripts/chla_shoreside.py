#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjørkelund 2021
# contact: tore.mo-bjorkelund@ntnu.no

import rospy
import numpy as np
from auv_handler import AuvHandler
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import os
import natural_cubic_spline
import scipy.spatial as scip
import pandas as pd
import time

import imc_ros_interface
from imc_ros_interface.msg import Chlorophyll, EstimatedState, Temperature, Salinity, Sms

class ChlaShoreside:
    def __init__(self):
        self.node_name = 'kongsfjorden_shoreside'
        rospy.init_node(self.node_name,anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        # connect to manta GW
        self.auv_handler = AuvHandler(self.node_name)

        self.iridium_pub_ = rospy.Publisher("/IMC/In/IridiumMsgTx",IridiumMsgTx , queue_size = 10)

        rospy.Subscriber("/IMC/Out/IridiumMsgRx", IridiumMsgRx, self.IridiumMsgRxCB)
        rospy.Subscriber("/IMC/Out/IridiumTxStatus", Iridium, self.IridiumTxStatusCB)

        ## GP constants
        self.measurements_chla_max = np.empty((50,50))
        self.measurements_depths   = np.empty((50,50))
        self.measurement_times     = np.empty((50,50))

        ## States
        self.last_msg_time = 0.0





    def IridiumMsgRxCB(self, msg):
        #content =  msg.data.data
        content = "52420027cc28542920286c6175762d686172616c64292031353a30333a3233202f2036332032362e3434393537302c2031302032302e383935373230202f20663a383120633a3838"




    def IridiumTxStatusCB(self,msg):
        offset_north = msg.lat.data - self.origin_rad[0]
        offset_east = msg.lon.data - self.origin_rad[1]
        circumference = 40075000.0
        N = offset_north * circumference / (2.0 * np.pi)
        E = offset_east * circumference * np.cos(self.origin_rad[0]) / (2.0 * np.pi)
        D = msg.depth.data
        self.vehicle_pos = [N, E, D]
        if not self.init:
            self.init = True
            self.ada_state = "init"
