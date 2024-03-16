#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjørkelund 2021
# contact: tore.mo-bjorkelund@ntnu.no

import rospy
import numpy as np
from auv_handler import AuvHandler
from pykrige.ok import OrdinaryKriging
import matplotlib.pyplot as plt

import imc_ros_interface
from imc_ros_interface.msg import EstimatedState, Temperature, Salinity

class GPExample:
    def __init__(self):
        self.node_name = 'gp_example'
        rospy.init_node(self.node_name,anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz
        config = "/home/tore/software/adaframe_ws/src/adaframe_examples/scripts/gp_example.ini"

        self.auv_handler = AuvHandler(self.node_name,config)

        rospy.Subscriber("/Vehicle/Out/Temperature_filtered", Temperature, self.TemperatureCB)
        rospy.Subscriber("/Vehicle/Out/EstimatedState_filtered", EstimatedState, self.EstimatedStateCB)

        self.speed = 1.9 #m/s
        self.depth = 0.0 #meters
        self.init = False
        self.vehicle_pos = [0,0,0]
        self.last_state = "unavailable"

        #Earths circumference
        self.circumference = 40075000

        ############ GP values ##############
        self.origin = np.array([63.44045785,10.35580542])
        self.origin_rad = self.origin*np.pi/180.0
        #make a rectangle, in meters, [north, east]
        self.offsets = np.array([200.0, 200.0])

        self.gp_resolution = 11
        self.meters_per_grid = self.offsets/self.gp_resolution
        self.xgrid = np.linspace(0,self.offsets[0],self.gp_resolution)
        self.ygrid = np.linspace(0,self.offsets[1],self.gp_resolution)
        self.iteration = 0
        self.measurements = []
        #Set the first WP in the middle of the box
        self.first_wp = self.offsets/2.0
        self.first_wp_rad = self.grid2wp(self.first_wp[0],self.first_wp[1])
        self.auv_handler.setWaypoint(self.first_wp_rad[0],self.first_wp_rad[1], speed = self.speed, z = self.depth)
        self.rate.sleep()

        while not self.init:
            self.rate.sleep()

        rospy.loginfo("initiated")


    def TemperatureCB(self, msg):
        if self.init:
            self.measurements.append([self.vehicle_pos,msg.value.data])


    def EstimatedStateCB(self,msg):
        offset_north = msg.lat.data - self.origin_rad[0]
        offset_east = msg.lon.data - self.origin_rad[1]
        circumference = 40075000.0
        N = offset_north * circumference / (2.0 * np.pi)
        E = offset_east * circumference * np.cos(self.origin_rad[0]) / (2.0 * np.pi)
        D = msg.z.data
        self.vehicle_pos = [N, E, D]
        if not self.init:
            self.init = True

    def grid2wp(self,x,y):
        lat = self.origin_rad[0] + x*np.pi*2.0/self.circumference
        lon = self.origin_rad[1] + y*np.pi*2.0/(self.circumference*np.cos(self.origin_rad[0]))
        return lat,lon

    def updateWP(self):
        values = np.zeros((self.gp_resolution**2,2))
        for measurement in self.measurements:
            x = int(np.floor(measurement[0][0]/self.meters_per_grid[0]))
            y = int(np.floor(measurement[0][1]/self.meters_per_grid[1]))
            values[max(0,min(x + y*self.gp_resolution,self.gp_resolution**2-1)),0] += measurement[1]
            values[max(0,min(x + y*self.gp_resolution,self.gp_resolution**2-1)),1] += 1

        x = []
        y = []
        temperature = []
        i = 0
        #Clear previous measurements
        self.measurements = []
        for value in values:
            if value[1] >= 1:
                temperature.append(value[0]/value[1])
                x.append((i%self.gp_resolution)*self.meters_per_grid[0])
                y.append(np.round(i/self.gp_resolution)*self.meters_per_grid[1])
                vpos = [(i%self.gp_resolution)*self.meters_per_grid[0],np.round(i/self.gp_resolution)*self.meters_per_grid[1],0.0]
                temperatureval = value[0]/value[1]
                self.measurements.append([vpos,temperatureval])
            i += 1

        # Create ordinary kriging object:
        OK = OrdinaryKriging(x,y,temperature,variogram_model="linear",verbose=True,enable_plotting=False)
        z1, ss1 = OK.execute("grid", self.xgrid, self.ygrid)

        fig1, ax1 = plt.subplots(1,2,figsize = (10,4))
        cs = ax1[0].contourf(self.ygrid ,self.xgrid, z1, cmap = "coolwarm")
        ax1[0].set_title('Interpolated Temperature [°C]')
        ax1[0].set_xlabel("Easting [m]")
        ax1[0].set_ylabel("Northing [m]")
        #ax1.clabel(cs, fontsize=9, inline=1)
        fig1.colorbar(cs,ax=ax1[0])
        #ax1[0].plot(y, x, "+w")
        art = ax1[1].contourf(self.ygrid,self.xgrid, ss1, cmap = "YlGn_r")
        ax1[1].set_title("Estimated Variance")
        ax1[1].set_xlabel("Easting [m]")
        ax1[1].set_ylabel("Northing [m]")
        plt.colorbar(art,ax=ax1[1])

        #change this to wherever you want your figures stored.
        pngstr = "/home/tore/software/adaframe_ws/figures/" + "gp_example_" + str(self.iteration) + ".png"
        plt.savefig(pngstr)

        #Choose the new waypoint
        i = 0
        maxvar = -100.0
        for val in ss1.flatten():
            if val > maxvar:
                maxvar = val
                x = (i%self.gp_resolution)*self.meters_per_grid[0]
                y = np.floor(i/self.gp_resolution)*self.meters_per_grid[1]
            i += 1

        lat,lon = self.grid2wp(x,y)
        self.auv_handler.setWaypoint(lat,lon)
        self.iteration += 1
        print("updatedWP")


    def run(self):
        while not rospy.is_shutdown():
            if self.init:
                if self.auv_handler.getState() == "waiting":
                    self.updateWP()
                self.last_state = self.auv_handler.getState()
                self.auv_handler.spin()
            self.rate.sleep()


if __name__ == "__main__":
    gpex = GPExample()
    try:
        gpex.run()
    except rospy.ROSInterruptException:
        pass
