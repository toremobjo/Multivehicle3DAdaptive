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
from imc_ros_interface.msg import EstimatedState, Temperature, Salinity, Sms

class DouroTracker:
    def __init__(self):
        self.node_name = 'douro_river'
        rospy.init_node(self.node_name,anonymous=True)
        self.rate = rospy.Rate(1) # 1Hz

        self.auv_handler = AuvHandler(self.node_name)

        #rospy.Subscriber("/Vehicle/Out/Temperature_filtered", Temperature, self.TemperatureCB)
        rospy.Subscriber("/Vehicle/Out/Salinity_filtered", Salinity, self.SalinityCB)
        rospy.Subscriber("/Vehicle/Out/EstimatedState_filtered", EstimatedState, self.EstimatedStateCB)

        self.sms_pub_ = rospy.Publisher("/IMC/In/Sms", Sms, queue_size = 10)


        self.speed = 1.9 #m/s
        self.depth = 10.0 #meters
        self.prior_length = 1000 #1600 # meters
        self.prior_headings = [155,290] # dregrees
        self.dive_angle = 15.0*np.pi/180.0
        self.padding = 35.0 #meters
        self.init = False
        self.prior_start = [41.14671048,-8.68872273]
        self.last_surface_time = rospy.get_time()
        self.mission_start_time = rospy.get_time()
        self.seconds_between_surface = 350
        self.seconds_on_surface = 40
        self.surface_next = False
        self.surface_depth = 0.25
        self.max_depth = 10.0
        self.depth_estimates = []
        self.lowpass_sal = 0
        self.alpha_sal = 0.9
        self.desired_z = 10.0
        self.desired_z_max = 10.0
        self.desired_z_min = 0.0
        self.river_mouth = [41.14155258*np.pi/180.0, -8.68198713*np.pi/180.0]
        self.river_mouth_angle_north = 330 * np.pi/180.0
        self.river_mouth_angle_south = np.pi
        self.curr_wp = [0,0]
        self.lowpass_depth = 3.0
        self.alpha_depth = 0.2
        self.mission_length = 1000000000 # seconds

        self.phone_number = rospy.get_param("/" + self.node_name + "/phone_number","+4790288012")
        self.max_depth = rospy.get_param("/" + self.node_name + "/max_depth", 10.0)
        self.desired_z = self.max_depth
        self.desired_z_max = self.max_depth
        self.mission_length = rospy.get_param("/" + self.node_name + "/mission_length", 36000)
        self.prior_start_lat = rospy.get_param("/" + self.node_name + "/starting_position_lat", 41.14671048)
        self.prior_start_lon = rospy.get_param("/" + self.node_name + "/starting_position_lon", -8.68872273)

        self.prior_start = [self.prior_start_lat, self.prior_start_lon]



        self.vehicle_pos = [0,0,0]
        self.last_state = "unavailable"
        self.ada_state = "init"

        log_name = "duoro_river" + str(rospy.get_time())
        log_directory = "/home/pi/adaframe_ws/logs/douro_river/"

        CHECK_FOLDER = os.path.isdir(log_directory)
        if not CHECK_FOLDER:
            os.makedirs(log_directory)
            print("created folder : ", log_directory)
        else:
            print(log_directory, "folder already exists.")

        #Earths circumference
        self.circumference = 40075000

        self.divelength  = self.depth/np.sin(self.dive_angle) + self.padding
        self.prior_wps = []
        last_wp = self.prior_start

        for ang in self.prior_headings:
            for i in range(int(np.floor(self.prior_length/self.divelength))):
                wp = [last_wp[0] + self.divelength*(i+1)*np.cos(ang*np.pi/180.0)*360.0/self.circumference, last_wp[1] + self.divelength*(i+1)*np.sin(ang*np.pi/180.0)*360.0/(self.circumference*np.cos(self.prior_start[0]*np.pi/180.0)),((i+1)%2)*self.depth]
                self.prior_wps.append(wp)
            last_wp = wp

        self.prior_end = [last_wp[0] + self.divelength*np.cos(self.prior_headings[-1]*np.pi/180.0)*360.0/self.circumference, last_wp[1] + self.divelength*np.sin(self.prior_headings[-1]*np.pi/180.0)*360.0/(self.circumference*np.cos(self.prior_start[0]*np.pi/180.0)),0.0]


        ############ GP values ##############
        self.origin = np.array([41.12696217,-8.7107383])
        self.origin_rad = self.origin*np.pi/180.0
        #make a rectangle, in meters, [north, east]
        self.offsets = np.array([5000.0, 5000.0])

        self.gp_resolution = 4
        self.meters_per_grid = self.offsets/(2*self.gp_resolution)
        self.max_path_angle = 90*np.pi/180.0
        self.iteration = 0
        self.measurements = []

        #Set the first WP at the start of the prior
        self.auv_handler.setWaypoint(self.prior_start[0]*np.pi/180.0,self.prior_start[1]*np.pi/180.0)
        self.rate.sleep()

        while not self.init:
            self.rate.sleep()

        self.generateEvalPts()
        self.generatePaths()

        rospy.loginfo("initiated")

    # Covariance function
    def getcov(self,d_sites, p_sites, par):
        sig2 = par[0] ** 2
        crange = par[1]
        h = scip.distance.cdist(d_sites, p_sites, 'sqeuclidean')

        for i in np.nditer(h, op_flags=['readwrite']):  # Modifying array values
            i[...] = sig2 * np.exp(-3 * (1.0 / crange) * np.sqrt(i))

        return np.transpose(h)

    def Matern_cov(self,d_sites, p_sites, par):
        sigma = par[0]
        eta = par[1]
        t = scip.distance.cdist(d_sites, p_sites, 'sqeuclidean')
        '''
        :param sigma: scaling coef
        :param eta: range coef
        :param t: distance matrix
        :return: matern covariance
        '''
        return sigma ** 2 * (1 + eta * t) * np.exp(-eta * t)


    def SalinityCB(self, msg):
        if self.init:
            self.measurements.append([self.vehicle_pos,msg.value.data])
            if self.lowpass_sal == 0:
                self.lowpass_sal = msg.value.data
            else:
                self.lowpass_sal = self.alpha_sal*self.lowpass_sal + (1.0-self.alpha_sal)*msg.value.data


    def EstimatedStateCB(self,msg):
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

    def grid2wp(self,x,y):
        lat = self.origin_rad[0] + x*np.pi*2.0/self.circumference
        lon = self.origin_rad[1] + y*np.pi*2.0/(self.circumference*np.cos(self.origin_rad[0]))
        return lat,lon

    def generateEvalPts(self):
        origin = self.vehicle_pos
        self.xgrid = np.linspace(-np.sqrt(self.offsets[0]/2),np.sqrt(self.offsets[0]/2),self.gp_resolution*2)
        self.xgrid = np.square(self.xgrid)*np.sign(self.xgrid)
        self.xgrid = np.concatenate((self.xgrid,self.xgrid,np.zeros(self.gp_resolution*2),self.xgrid))

        self.ygrid = np.linspace(-np.sqrt(self.offsets[1]/2),np.sqrt(self.offsets[1]/2),self.gp_resolution*2)
        self.ygrid = np.square(self.ygrid)*np.sign(self.ygrid)
        self.ygrid = np.concatenate((self.ygrid,-self.ygrid,self.ygrid,np.zeros(self.gp_resolution*2)))


        self.distance = np.linalg.norm(np.array([self.xgrid,self.ygrid]),axis = 0)

        tu = {"x" : self.xgrid, "y" : self.ygrid, "d" : self.distance}
        df = pd.DataFrame(tu)
        df.sort_values("d",inplace = True)

        self.xgrid = np.array(df["x"].values)
        self.ygrid = np.array(df["y"].values)
        self.distance = np.array(df["d"].values)


    def generatePaths(self):
        origin = [np.mean(self.xgrid),np.mean(self.ygrid)]
        paths = []
        for i in range(self.gp_resolution):
            pp = []
            for k in range(8**self.gp_resolution):
                pp.append(np.floor(k/(8**(i)))%8 +i*8)
            paths.append(pp)

        paths =  np.transpose(np.array(paths).astype(int))
        sorted_paths = []
        for path in paths:
            curr_origin = origin
            keep = True
            first = True
            for i in path:
                if not first:
                    v1 = np.array([self.ygrid[i]-curr_origin[1],self.xgrid[i]-curr_origin[0]])
                    v2 = np.array([curr_origin[1]-last_origin[1],curr_origin[0]-last_origin[0]])
                    n_v1 = np.linalg.norm(v1)
                    n_v2 = np.linalg.norm(v2)
                    dp = np.dot(v1/n_v1,v2/n_v2)
                    if dp < 0.0:
                        keep = False
                    elif dp <= 1.0:
                        if abs(np.arccos(dp)) > self.max_path_angle:
                            keep = False
                    else:
                        keep = False
                first = False
                last_origin = curr_origin
                curr_origin = [self.xgrid[i],self.ygrid[i]]
            if keep:
                sorted_paths.append(path)

        lens = []
        for path in sorted_paths:
            len = 0
            for i in range(self.gp_resolution-1):
                len += np.linalg.norm([self.xgrid[i+1]-self.xgrid[i],self.ygrid[i+1],self.ygrid[i]])
            lens.append(len)
        self.paths = np.array(sorted_paths)
        self.lens = np.array(lens)



    def measureSgrad(self):
        depths = []
        salinities = []
        Ns = []
        Es = []
        #print(self.measurements)
        for val in self.measurements:
            if val[0][2] > self.surface_depth:
                depths.append(val[0][2])
                salinities.append(val[1])
                Ns.append(val[0][0])
                Es.append(val[0][1])

        self.measurements = []

        if len(depths) > 20:
            gradmax = [np.mean(Ns),np.mean(Es),rospy.get_time()]
            if min(salinities) > 33:
                gradmax.append(0.0)
            elif max(salinities) < 28:
                gradmax.append(self.max_depth)
            else:
                model = natural_cubic_spline.get_natural_cubic_spline_model(np.array(depths),salinities, minval = min(depths), maxval = max(depths), n_knots = 6)
                offsetlength = 100
                offsets = np.linspace(min(depths),max(depths),num = offsetlength)
                predictions = model.predict(offsets)
                predictions_dx = []
                for i in range(len(predictions)-1):
                    predictions_dx.append(abs((predictions[i+1] - predictions[i])/(offsets[i+1]-offsets[i])))
                index = np.argmax(predictions_dx)
                gradmax.append(offsets[index])

            self.depth_estimates.append(gradmax)
            print(gradmax)
            self.lowpass_depth = self.lowpass_depth*self.alpha_depth + (1-self.alpha_depth)*gradmax[-1]

    def is_illigal(self,wp):
        if wp[1] > self.river_mouth[1]:
            return True
        if wp[0] > self.river_mouth[0]:
            if wp[1] > np.tan(self.river_mouth_angle_north)*(wp[0]-self.river_mouth[0])/np.cos(wp[0]) + self.river_mouth[1]:
                return True
        if wp[0] <= self.river_mouth[0]:
            if wp[1] > np.tan(self.river_mouth_angle_south)*(wp[0]-self.river_mouth[0])/np.cos(wp[0]) + self.river_mouth[1]:
                return True
        return False

    def updateWP(self):
        if self.ada_state != "surface":
            self.measureSgrad()

        if rospy.get_time() - self.last_surface_time > self.seconds_between_surface:
            if self.ada_state == "adapting":
                self.ada_state = "surfacing"

        if self.ada_state == "init":
            print(self.iteration)
            self.auv_handler.setWaypoint(self.prior_wps[self.iteration][0]*np.pi/180.0,self.prior_wps[self.iteration][1]*np.pi/180.0,z=self.prior_wps[self.iteration][2])
            self.ada_state = "prior"
            self.curr_wp = [self.prior_wps[self.iteration][0]*np.pi/180.0,self.prior_wps[self.iteration][1]*np.pi/180.0]

        elif self.ada_state == "prior":
            if self.iteration  < len(self.prior_wps):
                self.auv_handler.setWaypoint(self.prior_wps[self.iteration][0]*np.pi/180.0,self.prior_wps[self.iteration][1]*np.pi/180.0,z=self.prior_wps[self.iteration][2])
                self.curr_wp = [self.prior_wps[self.iteration][0]*np.pi/180.0,self.prior_wps[self.iteration][1]*np.pi/180.0]
            else:
                self.auv_handler.setWaypoint(self.prior_end[0]*np.pi/180.0,self.prior_end[1]*np.pi/180.0, z = 0.0)
                self.curr_wp = [self.prior_end[0]*np.pi/180.0,self.prior_end[1]*np.pi/180.0]
                self.ada_state = "adapting"
                print("last WP")

        elif self.ada_state == "adapting":
            if len(self.depth_estimates) is not 0:
                x = []
                y = []
                depths = []
                for val in self.depth_estimates:
                    x.append(val[0])
                    y.append(val[1])
                    depths.append(val[3])
                # ===== Find desired depth
                sigma = np.sqrt(np.var(depths[-20:-1]))
                if sigma < 1.5:
                    sigma = 1.5

                if self.iteration%2:
                    self.desired_z = self.lowpass_depth + 2.0*sigma
                    if self.desired_z > self.desired_z_max:
                        self.desired_z = self.desired_z_max
                else:
                    self.desired_z = self.lowpass_depth - 2.0*sigma
                    if self.desired_z < self.desired_z_min:
                        self.desired_z = self.desired_z_min
                # ===== Data sites
                x = np.array(x)
                y = np.array(y)
                depths = np.array(depths)
                data_sites = np.c_[x,y]
                # ===== Prediction sites
                pred_sites = np.c_[self.xgrid + self.vehicle_pos[0], self.ygrid + self.vehicle_pos[1]]
                # ===== Covariance calculation
                #cov_param = [0.4, 1000]
                cov_param = [ sigma, 0.001]
                X = np.c_[ np.ones(len(x)),x,y]

                # Calculate the multiple linear regression - y response is scalar temperature (in cont. to multivariate vec. resp)
                b = np.linalg.lstsq(X, depths, rcond=None)[0]
                # ===== Regression generation of prior mean

                mu = b[0] + b[1] * x + b[2] * y
                # Weights for distance to predictions
                d_depths = depths - mu
                t = rospy.get_time()
                k_bs = self.Matern_cov(data_sites, pred_sites, cov_param)
                k_sb = self.Matern_cov(pred_sites,data_sites, cov_param)
                k_ss = self.Matern_cov(pred_sites,pred_sites, cov_param)
                # Weights for similarity of measurements
                k_bb = self.Matern_cov(data_sites, data_sites, cov_param)
                tau = 0.005
                k_bb += tau ** 2 * np.eye(np.shape(k_bb)[0], np.shape(k_bb)[1])


                x_star = np.array(self.xgrid + self.vehicle_pos[0])
                y_star = np.array(self.ygrid + self.vehicle_pos[1])
                pred = b[0] + b[1] * x_star + b[2] * y_star
                pred = np.array(pred)

                pred_field =   k_sb @ np.linalg.inv(k_bb) @ d_depths + pred
                for i in range(len(pred_field)):
                    if pred_field[i] > self.max_depth:
                        pred_field[i] = self.max_depth
                    elif pred_field[i] < 0.0:
                        pred_field[i] = 0.0

                resulting_cov = k_ss - k_sb  @ np.linalg.inv(k_bb) @ k_bs
                #print(resulting_cov)

                #plt.scatter(x_star,y_star,c = pred_field)
                #plt.show()
                #plt.matshow(resulting_cov)
                #plt.show()

                scores = []
                self.k_l = - 0.001
                self.k_var = 3
                self.k_depth = 4 #3
                depth_intercept = 2
                c = 0
                for path in self.paths:
                    score = self.lens[c]*self.k_l
                    c+=1

                    for i in path:
                        score += self.k_var*resulting_cov[i][i]
                        score += self.k_depth*(1/(1 + np.exp(-pred_field[i]-self.lowpass_depth)))

                        wpx = self.xgrid[i] + self.vehicle_pos[0]
                        wpy = self.ygrid[i] + self.vehicle_pos[1]
                        if self.is_illigal(self.grid2wp(wpx,wpy)):
                            score -= 1000000.0

                    scores.append(score)

                best = np.argmax(scores)
                if max(scores) < 0.0:
                    rospy.signal_shutdown("fatal, on land or something similar")
                print(scores[best])
                best_path = self.paths[best]
                print(best_path)
                wpx = self.xgrid[best_path[0]] + self.vehicle_pos[0]
                wpy = self.ygrid[best_path[0]] + self.vehicle_pos[1]

                wp = self.grid2wp(wpx,wpy)
                #depth = (self.desired_z*(self.iteration%2))
                print(self.iteration)
                self.curr_wp = wp
                self.auv_handler.setWaypoint(wp[0],wp[1], z = self.desired_z)

        elif self.ada_state == "surfacing":
            self.auv_handler.setWaypoint(self.curr_wp[0],self.curr_wp[1],z=0.0)
            self.last_surface_time = rospy.get_time()
            self.surfacing_time = rospy.get_time()
            self.ada_state = "surface"
            print("surfacing")
            msg = Sms()
            msg.number.data = self.phone_number
            msg.timeout.data = 60
            contents = "Lauv position: "
            wpx =  self.vehicle_pos[0]
            wpy =  self.vehicle_pos[1]
            wp = self.grid2wp(wpx,wpy)
            msg.contents.data = contents + str(wp[0]*180.0/np.pi)[0:10] + ", " + str(wp[1]*180.0/np.pi)[0:10]
            self.sms_pub_.publish(msg)

            time.sleep(1)

        if self.ada_state == "surface":
            #self.auv_handler.setWaypoint(self.curr_wp[0],self.curr_wp[1],z=0.0)
            self.last_surface_time = rospy.get_time()
            self.auv_handler.setWaypoint(self.curr_wp[0],self.curr_wp[1],z=0.0)
            if rospy.get_time() - self.surfacing_time > self.seconds_on_surface:
                self.ada_state = "adapting"

        print(self.ada_state)

        self.iteration += 1


    def run(self):
        while not rospy.is_shutdown():
            if self.init:
                if self.auv_handler.getState() == "waiting":
                    self.updateWP()

                if self.auv_handler.getProximity() == "z_near" and self.ada_state == "adapting" and self.auv_handler.running_follow_ref:
                    self.updateWP()

                if rospy.get_time() - self.mission_start_time > self.mission_length:
                    rospy.signal_shutdown("mission ended due to timeout")

                self.last_state = self.auv_handler.getState()
                self.auv_handler.spin()
            self.rate.sleep()


if __name__ == "__main__":
    gpex = DouroTracker()
    try:
        gpex.run()
    except rospy.ROSInterruptException:
        pass
