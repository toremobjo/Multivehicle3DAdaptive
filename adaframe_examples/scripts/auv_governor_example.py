#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjørkelund 2021
# contact: tore.mo-bjorkelund@ntnu.no

import rospy
import numpy as np
from auv_handler import AuvHandler

import imc_ros_interface
from imc_ros_interface.msg import EstimatedState, Temperature, Waypoint

class AUVGovernorExample:
    def __init__(self):
        self.init = False
        self.node_name = 'auv_governor'
        rospy.init_node('auv_governor',anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        # Initiate the auv handler
        self.brex = AuvHandler(self.node_name,"hello")

        rospy.Subscriber("/Vehicle/Out/Temperature_filtered", Temperature, self.TemperatureCB)
        rospy.Subscriber("/Vehicle/Out/EstimatedState_filtered", EstimatedState, self.EstimatedStateCB)

        self.origin = None
        #make a rectangle, in meters, [north, east]
        self.offsets = [100.0, 100.0]
        self.speed = 1.3 #m/s
        self.depth = 0 #meters
        self.corner = 0
        self.acceptence_raduis = 25.0 #meters

        self.currentTemperature = None
        self.vehicle_pos = [0.0, 0.0, 0.0] # NED from origin in meters

        self.vehicle_state_timeout = 5.0
        self.vehicle_state_update_time  = None

        self.tries_to_get_origin = 100
        tries = 0
        rospy.loginfo('Waiting for first estimated state message...')
        while not self.origin and tries < self.tries_to_get_origin:
            self.rate.sleep()
            tries+=1
            if tries >= self.tries_to_get_origin:
                rospy.logerr("Vehicle not connected, try again")
        rospy.loginfo('Recived estimated state message; good to go.')
        self.offsets_rad = [2.0 * np.pi * self.offsets[0] / 40075000.0 ,
                            2.0 * np.pi * self.offsets[1] / (40075000.0*np.cos(self.origin[0]))]

        self.waypoints = []
        self.corners = []
        for i in range(0,2):
            for j in range(0,2):
                wpt = Waypoint()
                wpt.speed.data = self.speed
                wpt.z.data = self.depth
                wpt.lat.data = self.origin[0] + self.offsets_rad[0]*i
                wpt.lon.data = self.origin[1] + self.offsets_rad[1]*j
                self.corners.append([self.offsets[0]*i,self.offsets[1]*j])
                self.waypoints.append(wpt)

        self.init = True


    def EstimatedStateCB(self,msg):
            if not self.origin:
                self.origin = [msg.lat.data,msg.lon.data]
            elif self.init:
                offset_north = msg.lat.data - self.origin[0]
                offset_east = msg.lon.data - self.origin[1]
                circumference = 40075000.0
                N = offset_north * circumference / (2.0 * np.pi)
                E = offset_east * circumference * np.cos(self.origin[0]) / (2.0 * np.pi)
                D = msg.z.data
                self.vehicle_pos = [N, E, D]


    def TemperatureCB(self, msg):
        self.currentTemperature = msg.value.data

    def AuvExecute(self):
        while not rospy.is_shutdown():
            if self.init:
                if self.brex.getState() == "waiting":
                    self.corner += 1
                    if self.corner >= 4:
                        self.corner = 0
                self.brex.setWaypoint(self.waypoints[self.corner].lat.data,self.waypoints[self.corner].lon.data)
                self.brex.spin()#self.waypoint_publisher.publish(self.waypoints[self.corner])
            self.rate.sleep()

if __name__ == '__main__':
    ag = AUVGovernorExample()
    try:
        ag.AuvExecute()
    except rospy.ROSInterruptException:
        pass #TODO Send abort if this one fails
