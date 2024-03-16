#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjørkelund 2022
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
import SpatialLogGP
import copy
import struct
import math
from datetime import datetime

import imc_ros_interface
from imc_ros_interface.msg import Chlorophyll, EstimatedState, Temperature, Salinity, Sms, IridiumTxStatus, IridiumMsgRx
from std_msgs.msg import Byte, Float64

class ChlaTracker:
    def __init__(self):
        self.init = False
        self.node_name = 'kongsfjorden_shoreside'
        rospy.init_node(self.node_name,anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        self.auv_handler = AuvHandler(self.node_name)

        self.phone_number               = rospy.get_param("/" + self.node_name + "/phone_number","+4790288012")
        self.max_depth                  = rospy.get_param("/" + self.node_name + "/max_depth", 10.0)
        self.seconds_between_surface    = rospy.get_param("/" + self.node_name + "/popup_interval", 60.0)
        self.iridium_destination        = rospy.get_param("/" + self.node_name + "/iridium_destination","manta-ntnu-1")
        self.origin_lat                 = rospy.get_param("/" + self.node_name + "/origin_lat", 78.93630313)
        self.origin_lon                 = rospy.get_param("/" + self.node_name + "/origin_lon", 11.95967777)
        self.no_vehicles                = rospy.get_param("/" + self.node_name + "/no_vehicles", 2)
        self.vehicle_no                 = rospy.get_param("/" + self.node_name + "/vehicle_no", 99)
        self.op_orientation             = rospy.get_param("/" + self.node_name + "/op_orientation", 0)
        self.op_orientation             = self.op_orientation*np.pi/180.0

        x_extent = rospy.get_param("/" + self.node_name + "/x_extent", 200)
        y_extent = rospy.get_param("/" + self.node_name + "/y_extent", 200)
        z_extent = rospy.get_param("/" + self.node_name + "/z_extent", 10)
        self.grid_extent = [x_extent,y_extent,z_extent]

        x_grids = rospy.get_param("/" + self.node_name + "/x_grids", 5)
        y_grids = rospy.get_param("/" + self.node_name + "/y_grids", 5)
        z_grids = rospy.get_param("/" + self.node_name + "/z_grids", 5)
        self.grid_size = [x_grids,y_grids,z_grids]

        x_lscale = rospy.get_param("/" + self.node_name + "/x_lscale", 600)
        y_lscale = rospy.get_param("/" + self.node_name + "/y_lscale", 600)
        z_lscale = rospy.get_param("/" + self.node_name + "/z_lscale", 3)
        self.lscales = [x_lscale,y_lscale,z_lscale]

        self.sigma2 = rospy.get_param("/" + self.node_name + "/sigma2", 0.4)
        self.nugget = rospy.get_param("/" + self.node_name + "/nugget", 0.1)
        self.time_sat = rospy.get_param("/" + self.node_name + "/time_sat", 20000)

        self.start_hour_utc = rospy.get_param("/" + self.node_name + "/start_hour_utc", 6)

        self.gp = SpatialLogGP.GriddedLogGaussianProcess3D(self.lscales,
                        self.grid_size, self.grid_extent,self.sigma2,
                        time_sat=self.time_sat,nugget=self.nugget)

        self.last_state = "unavailable"
        self.ada_state = "init"

        self.log_name = "kingsbay_shoreside" + str(rospy.get_time())
        self.log_directory = "/home/tore/software/adaframe_ws/logs/kongsfjorden/"

        CHECK_FOLDER = os.path.isdir(self.log_directory)
        if not CHECK_FOLDER:
            os.makedirs(self.log_directory)
            print("created folder: ", self.log_directory)
        else:
            print(self.log_directory, "folder already exists.")

        self.origin         = np.array([self.origin_lat,self.origin_lon])
        self.origin_rad     = self.origin*np.pi/180.0
        self.vehicle_pos    = [0.0,0.0,0.0]
        self.boat_lat = 0.0
        self.boat_lon = 0.0
        self.boat_pos = [self.boat_lat,self.boat_lon]

        self.mission_start_time = rospy.get_time()
        self.reference_time = datetime.now().replace(hour=self.start_hour_utc, minute=0, second=0, microsecond=0).timestamp()

        self.df = pd.DataFrame(columns=["d", "c", "t"])
        self.dfs_to_send = []

        #for i in range(self.no_vehicles):
        #    self.dfs_to_send(pd.DataFrame(columns=["d", "c", "t"]))

        self.on_board_dfs = []
        self.other_vpos = []
        for i in range(self.no_vehicles):
            self.dfs_to_send.append(pd.DataFrame(columns=["d", "c", "t"]))
            self.other_vpos.append([0.0,0.0])


        self.iridium_req_id = 0
        self.iridium_status = "ready"
        self.send_timeout = 120

        self.last_surface_time = rospy.get_time()

        self.suspended  = False
        self.rate.sleep()

        rospy.Subscriber("/Vehicle/Out/EstimatedState_filtered", EstimatedState, self.EstimatedStateCB)
        rospy.Subscriber("/IMC/Out/IridiumTxStatus", IridiumTxStatus, self.IridiumTxStatusCB)
        rospy.Subscriber("/IMC/Out/IridiumMsgRx", IridiumMsgRx, self.IridiumMsgRxCB)
        rospy.Subscriber("boat_lat",Float64, self.boat_latCB)
        rospy.Subscriber("boat_lon",Float64, self.boat_lonCB)

        self.sms_pub_ = rospy.Publisher("/IMC/In/Sms", Sms, queue_size = 10)

        while not self.init:
            self.rate.sleep()

        rospy.loginfo("initiated")
        self.auv_handler.spin()

    def boat_latCB(self,msg):
        self.boat_pos[0] = msg.data*np.pi/180.0
        print("Boat_pos:")
        print(self.boat_pos)

    def boat_lonCB(self,msg):
        self.boat_pos[1] = msg.data*np.pi/180.0
        print("Boat_pos:")
        print(self.boat_pos)

    def IridiumTxStatusCB(self,msg):
        if msg.req_id.data == self.iridium_req_id-1:
            if msg.status.data == 1 or  msg.status.data == 5 or msg.status.data == 6:
                self.iridium_status = "ready"
            elif msg.status.data == 2:
                self.iridium_status = "error"
            elif msg.status.data == 3 or msg.status.data == 4:
                self.iridium_status = "busy"
        else:
            rospy.loginfo("Wrong iridium req id")
        print(self.iridium_status)

    def IridiumMsgRxCB(self,msg):
        rospy.loginfo("Got Iridium Message: %s", msg.data.data)
        origin = msg.origin.data
        self.waiting_for_iridium_data = False
        dd = bytes.fromhex(str(msg.data.data))
        bb = []
        for i in range(len(dd)):
            bb.append(struct.unpack("b",dd[i:i+1])[0]+128)

        dd = struct.pack("B",bb[0])
        for b in bb[1:]:
            dd+=struct.pack("B",b)
        print(dd)
        msgid = struct.unpack("B",dd[0])
        if msgid is 4:
            tau = struct.unpack("f",dd[1:5])[0]+self.reference_time
            vno = struct.unpack("B",dd[5:6])[0]
            lat = struct.unpack("f",dd[6:10])[0]
            lon = struct.unpack("f",dd[10:14])[0]
            lenth = struct.unpack("B",dd[14:15])[0]
            t = []
            c = []
            d = []
            for i in range(lenth):
                t.append(self.reference_time + struct.unpack("B",dd[15+i:16+i])[0]/0.003)
                c.append(struct.unpack("B",dd[15+i+lenth:16+i+lenth])[0]/10.0)
                d.append(struct.unpack("B",dd[15+i+2*lenth:16+i+2*lenth])[0]/10.0)

            df_from_v = pd.DataFrame({"d":d, "c":c, "t":t})

            df_to_send = self.df[~self.df1.apply(tuple,1).isin(self.on_board_dfs[vno].apply(tuple,1))]

            bb  = struct.pack("B",99)
            bb += struct.pack("f",rospy.get_time()-self.reference_time)
            bb += struct.pack("f",self.other_vpos[(vno+1)%2][0])
            bb += struct.pack("f",self.other_vpos[(vno+1)%2][1])
            bb += struct.pack("f",self.boat_pos[0])
            bb += struct.pack("f",self.boat_pos[1])

            lenth = len(df_to_send["t"].values[-60:])
            bb += struct.pack("B",lenth)
            for t in df_to_send["t"].values[-60:]:
                bb += struct.pack("B",int((t-self.reference_time)*0.003)%256)
            for c in df_to_send["c"].values[-60:]:
                bb += struct.pack("H",c)
            for d in df_to_send["d"].values[-60:]:
                bb += struct.pack("B",min(int(d*10.0),255))

            dd = []
            for b in bb:
                dd.append(b-128)

            self.auv_handler.sendIridium(timeout = self.send_timeout, iridium_destination = origin ,data = dd, request_id = self.iridium_req_id)
            self.iridium_req_id += 1
            self.iridium_status = "busy"
            while rospy.get_time() - self.last_surface_time <= self.send_timeout or self.iridium_status == "ready":
                self.auv_handler.spin()

            ## assimilate and clean
            self.df = pd.concat([self.df,df_from_v])
            self.df.sort_values(by="t",inplace=True)
            self.df.drop_duplicates(subset="c",keep="last",inplace=True)

            self.on_board_dfs[vno] = pd.concat([self.on_board_dfs[vno],df_from_v])
            self.on_board_dfs[vno].sort_values(by="t",inplace=True)
            self.on_board_dfs[vno].drop_duplicates(subset="c",keep="last",inplace=True)


        if msgid is 5:
            vno = struct.unpack("B",dd[1])
            lat = struct.unpack("f",dd[2:6])
            lon = struct.unpack("f",dd[6:10])
            self.other_vpos[vno] = [lat,lon]


    def EstimatedStateCB(self,msg):
        offset_north = msg.lat.data - self.origin_rad[0]
        offset_east = msg.lon.data - self.origin_rad[1]
        circumference = 40075000.0
        N = offset_north * circumference / (2.0 * np.pi)
        E = offset_east * circumference * np.cos(self.origin_rad[0]) / (2.0 * np.pi)
        D = msg.depth.data
        xx = E * np.cos(self.op_orientation) - N * np.sin(self.op_orientation)
        yy = E * np.sin(self.op_orientation) + N * np.cos(self.op_orientation)
        self.vehicle_pos = [xx, yy, D]
        if not self.init:
            self.init = True
            self.ada_state = "init"

    def grid2wp(self,x,y):
        xx =  np.cos(self.op_orientation)*x + np.sin(self.op_orientation)*y
        yy = -np.sin(self.op_orientation)*x + np.cos(self.op_orientation)*y
        circumference = 40075000.0
        lat = self.origin_rad[0] + yy*np.pi*2.0/circumference
        lon = self.origin_rad[1] + xx*np.pi*2.0/(circumference*np.cos(self.origin_rad[0]))
        return lat,lon



    def run(self):
        while not rospy.is_shutdown():
            if self.init:
                self.auv_handler.spin()
            self.rate.sleep()


if __name__ == "__main__":
    gpex = ChlaTracker()
    try:
        gpex.run()
    except rospy.ROSInterruptException:
        pass
