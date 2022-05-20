import rospy
import rospkg
import numpy as np
import argparse
import time
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo, WaypointInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo

class VehicleDecision():
    def __init__(self):
        self.vehicle_state = 'straight'
        self.lane_state = 0
        self.counter = 0
        self.lane_counter = 0
        self.lane_counter_val = 40

        self.lane_marker = None
        self.target_x = None
        self.target_y = None
        self.change_lane = False
        self.change_lane_wp_idx = 0
        self.detect_dist = 30
        self.speed = 25

        self.reachEnd = False
        self.wp_array = []
        self.speed_s = 25
        self.speed_t = 24
        self.target_lane = 0
        self.change_lane = False

    def get_ref_state(self, currState, obstacleList, lane_marker, waypoint):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        # self.reachEnd = waypoint.reachedFinal
        lane_len = len(lane_marker.lane_markers_center.location)
        self.lane_marker = lane_marker.lane_markers_center.location[lane_len//2]
        self.lane_state = lane_marker.lane_state
        
        # calculating lane curve (+ left, - right)
        lane_dx = lane_marker.lane_markers_center.location[lane_len//2].x - lane_marker.lane_markers_center.location[0].x
        lane_dy = lane_marker.lane_markers_center.location[lane_len//2].y - lane_marker.lane_markers_center.location[0].y
        lane_dx2 = lane_marker.lane_markers_center.location[-1].x - lane_marker.lane_markers_center.location[lane_len//2].x 
        lane_dy2 = lane_marker.lane_markers_center.location[-1].y - lane_marker.lane_markers_center.location[lane_len//2].y
        angle1 =  np.arctan2(lane_dy2, lane_dx2) 
        angle2 = np.arctan2(lane_dy, lane_dx)
        
        if angle1 < 0 and angle2 >= 0:
            lane_curve = min(angle2 - angle1, np.abs(angle2 - (angle1 + np.pi*2)))
        elif angle1 >= 0 and angle2 < 0:
            lane_curve = -min(angle1 - angle2, np.abs(angle1 - (angle2 + np.pi*2)))
        else:
            lane_curve = angle2 - angle1
        curve_skew = lane_curve * 3.5
        print("cuve_skew: ", curve_skew)

        lane_dist = (lane_marker.lane_markers_center.location[0].x - lane_marker.lane_markers_center.location[-1].x)**2
        lane_dist += (lane_marker.lane_markers_center.location[0].y - lane_marker.lane_markers_center.location[-1].y)**2
        lane_dist = np.sqrt(lane_dist)
        print("dist: ", lane_dist)
        if lane_dist < 6:
            self.lane_marker = lane_marker.lane_markers_center.location[-1]
        elif lane_dist > 15:
            self.lane_marker = lane_marker.lane_markers_center.location[lane_len//3]
        
        if not self.target_x or not self.target_y:
            self.target_x = self.lane_marker.x
            self.target_y = self.lane_marker.y
        if self.reachEnd:
            return None
        # print("Reach end: ", self.reachEnd)
        currentEuler = currState[1]
        curr_x = currState[0][0]
        curr_y = currState[0][1]

        # Check whether any obstacles are in the front of the vehicle
        obs_front = False
        obs_left = False
        obs_right = False
        obs_sideL = False
        obs_sideR = False
        front_dist = 100
        is_pedestrian = False
        if obstacleList:
            if self.lane_counter > self.lane_counter_val:
                self.lane_counter = 41
            self.lane_counter_val = 40
            for obs in obstacleList:
                #print(obs)
                #if "pedestrian" in obs.obstacle_name:
                #    self.lane_counter_val = 20
                #    is_pedestrian = True
                for vertex in obs.vertices_locations:
                    dy = vertex.vertex_location.y - curr_y
                    dx = vertex.vertex_location.x - curr_x
                    yaw = currState[1][2]
                    rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
                    ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx
                    
                    psi = np.arctan(ry / rx)
                    #print("psi: ", psi)
                    if rx > 0:
                        tmp_dist =  np.sqrt(dy * dy + dx * dx)
                        if "pedestrian" in obs.obstacle_name and tmp_dist > 0 and tmp_dist < self.detect_dist:
                            self.lane_counter_val = 20
                            is_pedestrian = True

                        if tmp_dist < front_dist:
                            front_dist = tmp_dist
                        #print("front_dist: ", front_dist)
                        # print("detected object is at {} away and {} radians".format(front_dist, psi))
                        if psi < 0.1 - curve_skew and psi > -0.1 - curve_skew:
                            obs_front = True
                        elif psi > 0.1 - curve_skew and psi < np.pi/10 - curve_skew:
                            obs_right = True
                        elif psi < -0.1 - curve_skew and psi > -np.pi/10 - curve_skew:
                            obs_left = True
                        elif psi >= np.pi/10 - curve_skew:
                            obs_sideR = True
                        elif psi <= -np.pi/10 - curve_skew:
                            obs_sideL = True

        #print("pedestrian: ", is_pedestrian)
        # prev_vehicle_state = self.vehicle_state
        if obs_left and obs_front and obs_right and front_dist <= self.detect_dist:
            self.vehicle_state = "stop"
            self.lane_counter = 0

        elif obs_front and is_pedestrian and front_dist <=self.detect_dist and self.lane_counter > self.lane_counter_val:
            self.vehicle_state = "stop"
            self.lane_counter = 0

        elif self.lane_state == LaneInfo.LEFT_LANE:
            if self.lane_counter > self.lane_counter_val:  #  or not self.vehicle_state == "straight":
                if front_dist <= self.detect_dist and obs_front:
                    if not obs_right and not obs_sideR:
                        self.vehicle_state = "turn-right"
                        #self.lane_counter = 0
                    elif not obs_sideR:
                        self.vehicle_state = "turn-right-right"
                    #     self.lane_counter = 0
                    else:
                        self.vehicle_state = "stop"
                        self.lane_counter = 0
                else:
                    self.vehicle_state = "straight"
            else:
                #self.vehicle_state = "straight"
                self.lane_counter += 1

        elif self.lane_state == LaneInfo.RIGHT_LANE:
            if self.lane_counter > self.lane_counter_val: #  or not self.vehicle_state == "straight":
                if front_dist <= self.detect_dist and obs_front:
                    if not obs_left and not obs_sideL:
                        self.vehicle_state = "turn-left"
                        #self.lane_counter = 0
                    elif not obs_sideL:
                         self.vehicle_state = "turn-left-left"
                    #     self.lane_counter = 0
                    else:
                        self.vehicle_state = "stop"
                        self.lane_counter = 0
                else:
                    self.vehicle_state = "straight"
            else:
                #self.vehicle_state = "straight"
                self.lane_counter += 1

        elif self.lane_state == LaneInfo.CENTER_LANE:
            if self.lane_counter > self.lane_counter_val: #or not self.vehicle_state == "straight":
                if front_dist > self.detect_dist:
                    self.vehicle_state = "straight"
                else:
                    if not obs_front and not is_pedestrian:
                        self.vehicle_state = "straight"
                    elif not obs_right and not obs_sideR:
                        self.vehicle_state = "turn-right"
                        #self.lane_counter = 0
                    elif not obs_left and not obs_sideL:
                        self.vehicle_state = "turn-left"
                        #self.lane_counter = 0
                    else:
                       self.vehicle_state = "stop"
                       self.lane_counter = 0
            else:
                #self.vehicle_state = "straight"
                self.lane_counter += 1

        #if self.vehicle_state == "stop":
        #    self.speed = 1
        #else:
        #    self.speed = 25

        # print(front_dist, self.lane_state, self.vehicle_state, obs_front, obs_left, obs_right)

        #while not self.target_x or not self.target_y:
        #    continue
        dx = self.target_x - curr_x
        dy = self.target_y - curr_y
        xError = (self.target_x - curr_x) * np.cos(
            currentEuler[2]) + (self.target_y - curr_y) * np.sin(currentEuler[2])

        distToTargetX = abs(self.target_x - curr_x)
        distToTargetY = abs(self.target_y - curr_y)
        curr_v = np.sqrt(currState[2][0]**2 + currState[2][1]**2)

        if xError < 0 or (distToTargetX < 2 and distToTargetY < 2):
        #if xError < 1.25:
            #print("lane: ", self.lane_state)

            prev_target_x = self.target_x
            prev_target_y = self.target_y

            
            dx = self.lane_marker.y - prev_target_y
            dy = self.lane_marker.x - prev_target_x
            target_orientation = np.arctan2(self.lane_marker.y - prev_target_y,
                                            self.lane_marker.x - prev_target_x)

            scale_sh = 3.1#4.0
            scale_st = 3.1#2.0

            #curr_v = np.sqrt(currState[2][0]**2 + currState[2][1]**2)
            if self.vehicle_state == "stop":
                speed_d = -curr_v*2
                if curr_v < 10:
                    speed_d = curr_v
                self.wp_array.append((self.lane_marker.x, self.lane_marker.y, speed_d))
                
            if self.change_lane and self.target_lane == self.lane_state:
                self.change_lane = False
            # vehicle did not turn enough
            if self.change_lane and self.target_lane != self.lane_state and len(self.wp_array) == 0:
                self.lane_counter = 0
                
                # turn left
                if self.target_lane < self.lane_state:
                    x_offset_shallow = np.cos(target_orientation - np.pi/4)*scale_sh
                    y_offset_shallow = np.sin(target_orientation - np.pi/4)*scale_sh
                    self.wp_array.append((self.target_x + x_offset_shallow, self.target_y  + y_offset_shallow, self.speed_t))

                # turn right
                else:
                    x_offset_shallow = np.cos(target_orientation + np.pi/4)*scale_sh
                    y_offset_shallow = np.sin(target_orientation + np.pi/4)*scale_sh
                    self.wp_array.append((self.target_x  + x_offset_shallow, self.target_y +y_offset_shallow, self.speed_t))


            #elif self.vehicle_state == "straight" and len(self.wp_array) == 0:
            #    self.wp_array.append((self.lane_marker.x, self.lane_marker.y, self.speed_s))

            elif self.vehicle_state == "turn-right":
                # self.change_lane = False
                self.vehicle_state = "straight"
                self.lane_counter = 0
                self.change_lane = True
                self.target_lane = self.lane_state + 1
                
                base_x = self.lane_marker.x
                base_y = self.lane_marker.y
                curv_factor = lane_curve * 0.0
                
                x_offset_shallow = np.cos(target_orientation + np.pi/8 - curv_factor)*scale_sh
                y_offset_shallow = np.sin(target_orientation + np.pi/8 - curv_factor)*scale_sh
                x_offset_steep = np.cos(target_orientation + np.pi/6 - curv_factor)*scale_st
                y_offset_steep = np.sin(target_orientation + np.pi/6 - curv_factor)*scale_st
                self.wp_array.append((base_x  + x_offset_steep, base_y  + y_offset_steep, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_shallow, self.wp_array[-1][1] + y_offset_shallow, self.speed_t))
                #self.wp_array.append((self.wp_array[-1][0] + x_offset_shallow, self.wp_array[-1][0] + y_offset_shallow))

                #self.target_x = self.target_x + x_offset
                #self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "turn-left":
                # self.change_lane = False
                self.vehicle_state = "straight"
                self.lane_counter = 0
                self.change_lane = True
                self.target_lane = self.lane_state - 1
                
                base_x = self.lane_marker.x
                base_y = self.lane_marker.y
                quad = target_orientation//(np.pi/2)
                curv_factor = lane_curve * 0.0
                
                x_offset_shallow = np.cos(target_orientation - np.pi/8 + curv_factor)*scale_sh
                y_offset_shallow = np.sin(target_orientation - np.pi/8 + curv_factor)*scale_sh
                x_offset_steep = np.cos(target_orientation - np.pi/6 + curv_factor)*scale_st
                y_offset_steep = np.sin(target_orientation - np.pi/6 + curv_factor)*scale_st
                self.wp_array.append((base_x  + x_offset_steep, base_y  + y_offset_steep, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_shallow, self.wp_array[-1][1] + y_offset_shallow, self.speed_t))
                #self.wp_array.append((self.wp_array[-1][0] + x_offset_shallow, self.wp_array[-1][0] + y_offset_shallow))

                #self.target_x = self.target_x + x_offset
                #self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "turn-right-right":
                # self.change_lane = False
                self.vehicle_state = "straight"
                self.lane_counter = -20
                self.change_lane = True
                self.target_lane = self.lane_state + 2

                base_x = self.lane_marker.x
                base_y = self.lane_marker.y
                curv_factor = lane_curve * 0.0

                x_offset_shallow = np.cos(target_orientation + np.pi/8 + curv_factor)*scale_sh
                y_offset_shallow = np.sin(target_orientation + np.pi/8 + curv_factor)*scale_sh
                x_offset_steep = np.cos(target_orientation + np.pi/4 + curv_factor)*1.1*scale_st
                y_offset_steep = np.sin(target_orientation + np.pi/4 + curv_factor)*1.1*scale_st
                self.wp_array.append((base_x  + x_offset_steep, base_y  + y_offset_steep, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_steep, self.wp_array[-1][1] + y_offset_steep, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_shallow, self.wp_array[-1][1] + y_offset_shallow, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_shallow, self.wp_array[-1][1] + y_offset_shallow, self.speed_t))


            elif self.vehicle_state == "turn-left-left":
                # self.change_lane = False
                self.vehicle_state = "straight"
                self.lane_counter = -20
                self.change_lane = True
                self.target_lane = self.lane_state - 2

                base_x = self.lane_marker.x
                base_y = self.lane_marker.y
                quad = target_orientation//(np.pi/2)
                curv_factor = lane_curve * 0.0

                x_offset_shallow = np.cos(target_orientation - np.pi/8 + curv_factor)*scale_sh
                y_offset_shallow = np.sin(target_orientation - np.pi/8 + curv_factor)*scale_sh
                x_offset_steep = np.cos(target_orientation - np.pi/4 + curv_factor)*1.1*scale_st
                y_offset_steep = np.sin(target_orientation - np.pi/4 + curv_factor)*1.1*scale_st
                self.wp_array.append((base_x  + x_offset_steep, base_y  + y_offset_steep, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_steep, self.wp_array[-1][1] + y_offset_steep, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_shallow, self.wp_array[-1][1] + y_offset_shallow, self.speed_t))
                self.wp_array.append((self.wp_array[-1][0]+ x_offset_shallow, self.wp_array[-1][1] + y_offset_shallow, self.speed_t))

            elif len(self.wp_array) == 0:
                if curve_skew < -0.7 and (obs_left or obs_sideL):
                    base_x = self.lane_marker.x
                    base_y = self.lane_marker.y
                    x_offset_shallow = np.cos(target_orientation + np.pi/6)*scale_sh
                    y_offset_shallow = np.sin(target_orientation + np.pi/6)*scale_sh
                    self.wp_array.append((base_x  + x_offset_shallow, base_y  + y_offset_shallow, self.speed_t))
                elif curve_skew > 0.7 and (obs_right or obs_sideR):
                    base_x = self.lane_marker.x
                    base_y = self.lane_marker.y
                    x_offset_shallow = np.cos(target_orientation - np.pi/6)*scale_sh
                    y_offset_shallow = np.sin(target_orientation - np.pi/6)*scale_sh
                    self.wp_array.append((base_x  + x_offset_shallow, base_y  + y_offset_shallow, self.speed_t))
                else:
                    self.wp_array.append((self.lane_marker.x, self.lane_marker.y, self.speed_s))
    
            #print("wp: ", self.wp_array)
            if len(self.wp_array) > 0:
                new_point = self.wp_array.pop(0)
                self.target_x = new_point[0]
                self.target_y = new_point[1]
                self.speed = new_point[2]
        else:
            self.counter += 1
        print("state: ", self.vehicle_state)
        print("lane: ", self.lane_state)
        print("counter: ", self.lane_counter)
        #wp_array = [[self.target_x, self.target_y, self.speed],]
        #print("current_wp:", [self.target_x, self.target_y, self.speed])
        print("obs: ", obs_sideL, obs_left, obs_front, obs_right, obs_sideR)
        #print(obstacleList)
        return [self.target_x, self.target_y, self.speed]

class VehicleController():
    def __init__(self):
        self.y_accum = 0
        self.x_accum = 0
    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = currentPose[1]
        curr_x = currentPose[0][0]
        curr_y = currentPose[0][1]

        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]
        #P
        k_s = 0.05#.05
        k_n = 0.8#1.1
        #D
        k_ds = 1.3#1.5
        k_theta = 1.65 # 1.5  1.0
        #I
        k_ix = 0.0 #0.0
        k_iy = 0.001 #.001
        
        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(
            currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        vError = target_v - curr_v
        
        theta_error = 0 - np.arctan2(yError, 1)
        #print("theta_error:", theta_error)
        # calculating lane curve (+ left, - right)
        
        angle1 = np.arctan2(dy, dx)
        angle2 = currentEuler[2]

        if angle1 < 0 and angle2 >= 0:
            tmp_theta = min(angle2 - angle1, np.abs(angle2 - (angle1 + np.pi*2)))
        elif angle1 >= 0 and angle2 < 0:
            tmp_theta = -min(angle1 - angle2, np.abs(angle1 - (angle2 + np.pi*2)))
        else:
            tmp_theta = angle2 - angle1
         
        #print("tmp_theta", tmp_theta)
        

        #tmp_theta = np.arctan2(dy, dx) - currentEuler[2]
        #tmp_theta = min(tmp_theta, np.pi*2 - tmp_theta)
        #print("tmp_theta:", np.arctan2(dy, dx), currentEuler[2])
        print("x_error: ", xError)
        #print("y_error:", yError)
        if curr_v > 1:
            self.y_accum += yError
            self.x_accum += xError
        

        delta = k_n * yError + k_theta * tmp_theta + k_iy * self.y_accum
        #print("steering: ", delta)
        # Checking if the vehicle need to stop
        if target_v > 0:
            v = xError * k_s + vError * k_ds + k_ix * self.x_accum
            #Send computed control input to vehicle
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = v
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd
        else:
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.acceleration = target_v*3.5
            newAckermannCmd.speed = 0
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd


class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()
        self.decisionModule = VehicleDecision()
        self.controlModule = VehicleController()

    def stop(self):
        return self.controlModule.stop()

    def execute(self, currState, obstacleList, lane_marker, waypoint):
        # Get the target state from decision module
        refState = self.decisionModule.get_ref_state(currState, obstacleList,
                                                     lane_marker, waypoint)
        if not refState:
            return None
        return self.controlModule.execute(currState, refState)





