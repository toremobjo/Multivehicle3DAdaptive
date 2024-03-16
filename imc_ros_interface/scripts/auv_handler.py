#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjrkelund 2021
# contact: tore.mo-bjorkelund@ntnu.no

import rospy
import numpy as np
import configparser

import imc_ros_interface
from imc_ros_interface.msg import *


class AuvHandler:
    def __init__(self,node_name,configpath = "null", popup_interval = 120):
        self.init = False
        self.sensor_entities = []
        self.running_follow_ref = False
        self.sensors_found = False
        self.is_connected = False
        self.is_connected_last = False
        self.has_recieved_reference = False
        self.entitylist_attempts = 10
        self.neptus_src = 0
        self.vehicle_state = "unavailable" # ready and executing are eligeble states
        self.vehicle_proximity = "far"
        self.eta = 0
        self.last_ref = 0
        self.popup_duration = 10
        self.popup_interval = popup_interval
        self.suspended = False
        self.iridium_req_id = 0
        self.iridium_destination = "blank"
        #Heartbeat handler
        self.last_heartbeat_time = 0.0 # Seconds

        #Set the rate, 1 Hz is enough
        self.rate = rospy.Rate(1) # 1hz

        self.sensors = []
        self.sensor_messages = []
        self.loiter_radius = 20.0
        self.heartbeat_timeout = 5.0

        #self.parseConfig(configpath)

        #Get the params from the bridge.launch file
        try:
            self.sensors = rospy.get_param("/" + node_name + "/sensors","[CTD,CTD]")
            self.sensors = self.sensors.replace("[","").replace("]","").split(",")

            self.sensor_messages = rospy.get_param("/" + node_name + "/sensor_messages","[Temperature,Salinity]")
            self.sensor_messages = self.sensor_messages.replace("[","").replace("]","").split(",")

            self.loiter_radius = rospy.get_param("/" + node_name + "/loiter_radius", 20.0)
            self.heartbeat_timeout = rospy.get_param("/" + node_name + "/heartbeat_timeout", 5.0)
        except rospy.ROSInterruptException:
            rospy.logerr("could not get parameters from launch file.")

        self.phone_number = rospy.get_param("/" + node_name + "/phone_number","+4790288012")
        self.iridium_destination = rospy.get_param("/" + node_name + "/iridium_destination","manta-ntnu-1")

        self.reference = Reference()
        self.reference.radius.data = self.loiter_radius
        self.reference.flags.data = 0x27

        #Publisher going to the vehicle
        self.ref_pub_ = rospy.Publisher("/IMC/In/Reference", Reference, queue_size = 1)
        self.entityList_pub_ = rospy.Publisher("/IMC/In/EntityList", EntityList, queue_size = 1)
        self.logBookEntry_pub_  = rospy.Publisher("/IMC/In/LogBookEntry", LogBookEntry, queue_size = 1)
        self.sms_pub_ = rospy.Publisher("/IMC/In/Sms", Sms, queue_size = 1)
        self.iridium_pub_ = rospy.Publisher("/IMC/In/IridiumMsgTx", IridiumMsgTx, queue_size = 1)
        #Subscribers to the vehicle messages
        rospy.Subscriber("/IMC/Out/EntityList", EntityList, self.EntityListCB)
        rospy.Subscriber("/IMC/Out/Heartbeat", Heartbeat, self.HeartbeatCB)
        rospy.Subscriber("/IMC/Out/Abort", Abort, self.AbortCB)
        rospy.Subscriber("/IMC/Out/VehicleState", VehicleState, self.VehicleStateCB)
        rospy.Subscriber("/IMC/Out/EstimatedState", EstimatedState, self.EstimatedStateCB )
        rospy.Subscriber("/IMC/Out/FollowRefState", FollowRefState, self.FollowRefStateCB)
        rospy.Subscriber("/IMC/Out/PlanControl", PlanControl, self.PlanControlCB)
        rospy.Subscriber("/IMC/Out/Temperature", Temperature, self.TemperatureCB)
        rospy.Subscriber("/IMC/Out/Salinity", Salinity, self.SalinityCB )

        #Publishers to the rest of the framework
        self.temperature_filtered_pub_ = rospy.Publisher("/Vehicle/Out/Temperature_filtered", Temperature,queue_size = 1 )
        self.salinity_filtered_pub_ = rospy.Publisher("/Vehicle/Out/Salinity_filtered", Salinity,queue_size = 1 )
        self.estimatedState_filtered_pub_ = rospy.Publisher("/Vehicle/Out/EstimatedState_filtered", EstimatedState, queue_size = 1)
        # Subscriber to the rest of the framework
        rospy.Subscriber("/Vehicle/In/Waypoint", Waypoint, self.toReferenceCB)

        self.checkConfig()
        #self.AuvHandler()
        self.init = True

    def onConnect(self):
        self.findSensors()

    def onDisconnect(self):
        rospy.loginfo("Disconnected.")
        self.vehicle_state = "unavailable"
        self.running_follow_ref = False
        self.has_recieved_reference = False

    def parseConfig(self,configpath):
        config = configparser.ConfigParser()
        config.read(configpath)
        print(configpath)
        print(config.sections())
        self.sensors         = config["Sensor List"]["Sensors"]
        self.sensor_messages = config["Sensor List"]["Sensor Messages"]
        print(self.sensors)
        print(self.sensor_messages)

    def checkConfig(self):
        if not len(self.sensors) == len(self.sensor_messages):
            rospy.logerr("The sensors must correspond to the messages in the position in the array.")
            rospy.signal_shutdown("Change sensor entity array.")

    def findSensors(self):
        attempts = 0
        while not self.sensors_found and attempts < self.entitylist_attempts:
            msg = EntityList()
            msg.op.data = 1
            self.entityList_pub_.publish(msg)
            rospy.loginfo("dispatching entitylist query ")
            attempts +=1
            self.rate.sleep()


    def publishLogentry(self, what, type):
        logentry = LogBookEntry()
        logentry.type.data = type
        logentry.htime.data = rospy.get_time()
        logentry.context.data = "Adaptive mission"
        logentry.text.data = what
        self.logBookEntry_pub_.publish(logentry)

    def checkConnection(self):
        # Propagate last state for change detection
        self.is_connected_last = self.is_connected

        if rospy.get_time() - self.last_heartbeat_time < self.heartbeat_timeout:
            self.is_connected = True
            if not self.is_connected_last:
                self.onConnect()
            return True
        elif rospy.get_time() - self.last_heartbeat_time > self.heartbeat_timeout:
            self.is_connected = False
            rospy.loginfo("Waiting for connection")
            if self.is_connected_last:
                self.onDisconnect()
        return False

    def setWaypoint(self, lat, lon, z = 0.0, z_units = 1, speed = 1.5, speed_units = 0):
        self.has_recieved_reference = True
        self.reference.lat.data = lat
        self.reference.lon.data = lon
        self.reference.source.data = self.neptus_src
        self.reference.z.value.data = z
        self.reference.z.z_units.data = z_units
        self.reference.speed.value.data = speed
        self.reference.speed.speed_units.data = speed_units
        self.reference.header.timestamp.data = rospy.get_time()
        self.ref_pub_.publish(self.reference)
        try:
            msg = rospy.wait_for_message("/IMC/Out/FollowRefState", FollowRefState, timeout=5.0)
        except:
            pass

    def getState(self):
        return self.vehicle_state

    def getProximity(self):
        return self.vehicle_proximity

    def checkVehicleState(self):
        if self.checkConnection():
            if self.running_follow_ref and self.is_connected and self.has_recieved_reference:
                if self.vehicle_state is not "unavailable":
                    return True
        return False

    ########## CALLBACKS ##################
    def AbortCB(self,data):
        self.state = "unavailable"
        rospy.logwarn("The vehicle got an abort message, suspending ROS activity.")


    def EntityListCB(self,msg):
        if msg.op.data == 0:
            #rospy.loginfo("Got the entity list, parsing")
            entities = msg.list.data.split(";")
            self.sensor_entities = []
            for sensor in self.sensors:
                entity_found = False
                for ent in entities:
                    if sensor == ent.split("=")[0]:
                        entity_found = True
                        self.sensor_entities.append(int(ent.split("=")[1]))
                if not entity_found:
                    self.sensor_entities.append(None)
                    rospy.logwarn("Entity not discovered: %s.", sensor)
            self.sensors_found = True
            #rospy.loginfo("Entity List parsed ok.")
            #rospy.loginfo("Heard an entity list request.")

    def EstimatedStateCB(self,msg):
        circumference = 40075000.0
        eta = EstimatedState()
        eta = msg
        eta.lat.data = (msg.x.data * 2.0 * np.pi / circumference) + msg.lat.data
        eta.lon.data = (msg.y.data * 2.0 * np.pi / (circumference * np.cos(msg.lat.data))) + msg.lon.data
        if self.init:
            self.eta = eta
            self.estimatedState_filtered_pub_.publish(eta)

    def FollowRefStateCB(self,msg):
        if msg.state.data == 1:
            self.running_follow_ref = True
            self.vehicle_state = "starting"
        if msg.state.data == 2 or msg.state.data == 5:
            self.running_follow_ref = True
            self.vehicle_state = "executing"
        if msg.state.data == 3 or msg.state.data == 4:
            self.running_follow_ref = True
            self.vehicle_state = "waiting"
        if msg.state.data == 6:
            self.running_follow_ref = False
            self.vehicle_state = "unavailable"
            rospy.logwarn("System timed out.")

        if self.suspended:
            self.vehicle_state = "executing"
        bits = []
        val = msg.proximity.data
        for i in range(8):
            if val >= 2**(7-i):
                bits.append(1)
                val -= 2**(7-i)
            else:
                bits.append(0)
        bits = bits[::-1]
        if  bits[2] and bits[1]:
            self.vehicle_proximity = "xyz_near"
        elif bits[2]:
            self.vehicle_proximity = "z_near"
        else:
            self.vehicle_proximity = "far"



    def HeartbeatCB(self,msg):
        self.last_heartbeat_time = seconds = rospy.get_time() # Seconds

    def PlanControlCB(self,msg):
        if msg.type.data == 0 and msg.op.data == 0 and msg.plan_id.data == "follow_ntnu":
            self.running_follow_ref = True
            self.neptus_src = msg.source.data
            self.reference.source.data = self.neptus_src
        else:
            self.running_follow_ref = False


    def SalinityCB(self,msg):
        index = self.sensor_messages.index("Salinity")
        if self.sensors_found:
            if self.sensor_entities[index] == msg.header.src_ent.data:
                sal = Salinity()
                sal.header = msg.header
                sal.value.data = msg.value.data
                self.salinity_filtered_pub_.publish(sal)

    def TemperatureCB(self,msg):
        index = self.sensor_messages.index("Temperature")
        if self.sensors_found:
            if self.sensor_entities[index] == msg.header.src_ent.data:
                temp = Temperature()
                temp.header = msg.header
                temp.value.data = msg.value.data
                self.temperature_filtered_pub_.publish(temp)


    def toReferenceCB(self,msg):
        self.has_recieved_reference = True
        self.reference.lat.data = msg.lat.data #* np.pi / 180.0
        self.reference.lon.data = msg.lon.data #* np.pi / 180.0
        self.reference.z.value.data = msg.z.data
        self.reference.z.z_units.data = 1 # TODO
        self.reference.speed.value.data = 1.7 # TODO
        self.reference.speed.speed_units.data = 0 # TODO
        self.reference.source.data = self.neptus_src
        self.reference.header.timestamp.data = rospy.get_time()
        self.ref_pub_.publish(self.reference)


    def VehicleStateCB(self,msg):
        if msg.op_mode.data == 2:
            self.vehicle_state = "unavailable"
            rospy.logwarn("System encountered an error, restart FollowReference to continue.")
        if msg.op_mode.data == 5:
            self.vehicle_state = "unavailable"

    def PopUp(self,sms = True, iridium = False, popup_duration = 60, phone_number = "+4790288012", iridium_dest = "manta-ntnu-1"):
        if self.vehicle_state == "executing" or self.vehicle_state == "waiting":
            rospy.loginfo("Popping up.")
            self.suspended = True
            self.last_ref = self.reference
            self.reference.lat.data = self.eta.lat.data
            self.reference.lon.data = self.eta.lon.data
            self.reference.z.value.data = 0.0
            self.reference.z.z_units.data = 1 # TODO
            self.reference.speed.value.data = 1.7 # TODO
            self.reference.speed.speed_units.data = 0 # TODO
            self.reference.source.data = self.neptus_src
            self.popup_time = rospy.get_time()
            while self.vehicle_proximity != "z_near":
                self.spin()
            self.popup_time = rospy.get_time()
            if sms:
                self.sendSms(timeout = popup_duration,phone_number = phone_number)
            if iridium:
                self.sendIridium(timeout = popup_duration, iridium_destination = iridium_dest)
            while rospy.get_time() - self.popup_time <= popup_duration:
                self.spin()
            #self.reference = self.last_ref
            self.suspended = False
            self.spin()

    def sendSms(self,timeout = 60, phone_number = "+4790288012"):
        msg = Sms()
        msg.number.data = phone_number
        msg.timeout.data = timeout
        contents = "Lauv position: "
        px =  self.eta.lat.data
        py =  self.eta.lon.data
        wpx = self.last_ref.lat.data
        wpy = self.last_ref.lon.data
        msg.contents.data = contents + str(px) + "," + str(py) + " Next WP: " + str(wpx) + "," + str(wpy)
        self.sms_pub_.publish(msg)

    def sendIridium(self,iridium_destination = "manta-ntnu-1",timeout = 60, data = "hello", request_id = 0):
        msg = IridiumMsgTx()
        msg.req_id.data = request_id
        msg.ttl.data = timeout
        msg.destination.data = iridium_destination
        msg.data.data = data #contents + str(px) + "," + str(py) + " Next WP: " + str(wpx) + "," + str(wpy)
        self.iridium_pub_.publish(msg)

    def spin(self):
        #while not rospy.is_shutdown():
            #Check that we are connected, and that the vehicle is ready, else we wait.

        if self.checkVehicleState() or not rospy.is_shutdown():

            self.reference.header.timestamp.data = rospy.get_time()
            self.ref_pub_.publish(self.reference)
            #rospy.wait_for_message("/IMC/Out/FollowRefState", FollowRefState, timeout=None)
            self.rate.sleep()
        else:
            if rospy.is_shutdown():
                rospy.signal_shutdown("Going down")
            self.rate.sleep()
