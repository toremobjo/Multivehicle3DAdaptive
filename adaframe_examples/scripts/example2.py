#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjørkelund 2021
# contact: tore.mo-bjorkelund@ntnu.no

import rospy
import numpy as np
from auv_handler import AuvHandler

import imc_ros_interface
from imc_ros_interface.msg import Temperature

class EmptyExample:
    def __init__(self):
        self.node_name = 'empty_example'
        rospy.init_node(self.node_name,anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        self.auv_handler = AuvHandler(self.node_name,"empty")

        rospy.Subscriber("/Vehicle/Out/Temperature_filtered", Temperature, self.TemperatureCB)


        self.speed = 1.9 #m/s
        self.depth = 0.0 #meters
        self.lat = 63.45088257*np.pi/180.0
        self.lon = 10.38373793*np.pi/180.0
        self.last_state = "unavailable"
        self.rate.sleep()

        self.auv_handler.setWaypoint(self.lat,self.lon)
        self.init = True
        self.currentTemperature = 0.0

    def TemperatureCB(self,msg):
        self.currentTemperature = msg.value.data

    def run(self):
        while not rospy.is_shutdown():
            if self.init:
                if self.auv_handler.getState() == "waiting":
                    print("we are at Munkhomen")

                self.last_state = self.auv_handler.getState()
                self.auv_handler.spin()
            self.rate.sleep()


if __name__ == "__main__":
    gpex = EmptyExample()
    try:
        gpex.run()
    except rospy.ROSInterruptException:
        pass
