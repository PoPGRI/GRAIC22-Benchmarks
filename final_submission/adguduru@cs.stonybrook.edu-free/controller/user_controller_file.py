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
import math


class VehicleDecision():
    def __init__(self):
        self.vehicle_state = 'straight'
        self.lane_state = 0
        self.counter = 0

        self.lane_marker = None
        self.lane_marker = None

        self.target_x = None
        self.target_y = None
        self.change_lane = False
        self.change_lane_wp_idx = 0
        self.detect_dist = 60
        self.speed = 30
        self.target_lane = 4
        self.reachEnd = False
        self.rrt_counter = 0
        self.waypoints_rrt = []
        self.velocity_multiplier = 1
        self.pedestrian_loc = None
        self.pedestrian_angle = 0

    def calc_dist(self,l1, l2):
        return np.sqrt((l1.x-l2.x)**2 +(l1.y-l2.y)**2), l1.x-l2.x, l1.y-l2.y

    def rrt_main(self, currState, left_borders, right_borders, obstacle_list, left_extended, right_extended):
        # theta = np.arctan(currState[2][1]/(currState[2][0]+1e-9))
        theta = currState[1][2]
        # print("Rotation vector: ", currState[1])
        # print("RRT theta: ", theta, 180*theta)
        if theta<0:
            theta = 2*np.pi + theta
        start = np.array([currState[0][0], currState[0][1], theta])
        x_min = min(left_extended[0], left_borders[0][0], right_borders[0][0], right_extended[0], currState[0][0])
        x_max = max(left_extended[0], left_borders[0][0], right_borders[0][0], right_extended[0], currState[0][0])
        y_min = min(left_extended[1], left_borders[0][1], right_borders[0][1], right_extended[1], currState[0][1])
        y_max = max(left_extended[1], left_borders[0][1], right_borders[0][1], right_extended[1], currState[0][1])

        search_space = np.array([[x_min, x_max],[y_min,y_max],[0, 2*np.pi]])
        goal = (np.array(left_extended)+np.array(right_extended))/2
        goal = np.hstack((goal,theta))
        # print(goal)
        obstacles = []
        for obs in obstacle_list:
            center = np.array([obs.location.x, obs.location.y])
            radius = -np.inf
            for i in range(0, len(obs.vertices_locations),2):
                point = np.array([obs.vertices_locations[i].vertex_location.x, obs.vertices_locations[i].vertex_location.y])

                radius = max(radius, np.linalg.norm(center-point))
                obstacles.append(Obstacle(center, radius))
        rrtsearch = RRT(start, goal, obstacles, search_space, 0.01, 500, 0.03, '')
        path = rrtsearch.run()
        return path

    def calc_normal(self, curr, prev):
        if self.lane_state == 4:
            perp_left_factor = 3
        elif self.lane_state == 3:
            perp_left_factor = 1
        else:
            perp_left_factor = 5
        perp_right_factor = 6-perp_left_factor
        lane_vector = [curr.x - prev.x, curr.y - prev.y]
        norm_const = np.sqrt(lane_vector[0]**2 + lane_vector[1]**2)
        lane_left_normal = [lane_vector[1]/norm_const, -lane_vector[0]/norm_const]
        lane_right_normal = [-lane_vector[1]/norm_const, lane_vector[0]/norm_const]

        left_point = [lane_left_normal[0]*2.5*perp_left_factor + curr.x, lane_left_normal[1]*2.5*perp_left_factor + curr.y]
        right_point = [lane_right_normal[0]*2.5*perp_right_factor + curr.x, lane_right_normal[1]*2.5*perp_right_factor + curr.y]
        return left_point, right_point

    def get_borders(self, lane_marker):
        left_borders = []
        right_borders = []

        for i in range(1, len(lane_marker.lane_markers_center.location)):
            curr = lane_marker.lane_markers_center.location[i]
            prev = lane_marker.lane_markers_center.location[i-1]
            left_border, right_border = self.calc_normal(curr, prev)
            left_borders.append(left_border)
            right_borders.append(right_border)

        return np.array(left_borders), np.array(right_borders)

    def lengthSquare(self, X, Y):
        xDiff = X.x - Y.x
        yDiff = X.y - Y.y
        return xDiff * xDiff + yDiff * yDiff

    def printAngle(self, A, B, C):

        # Square of lengths be a2, b2, c2
        a2 = self.lengthSquare(B, C)
        b2 = self.lengthSquare(A, C)
        c2 = self.lengthSquare(A, B)

        # length of sides be a, b, c
        a = math.sqrt(a2);
        b = math.sqrt(b2);
        c = math.sqrt(c2);

        # From Cosine law
        alpha = math.acos((b2 + c2 - a2) /
                             (2 * b * c));
        betta = math.acos((a2 + c2 - b2) /
                             (2 * a * c));
        gamma = math.acos((a2 + b2 - c2) /
                             (2 * a * b));

        # Converting to degree
        alpha = alpha * 180 / math.pi;
        betta = betta * 180 / math.pi;
        gamma = gamma * 180 / math.pi;

        print("alpha : %f" %(alpha))
        print("betta : %f" %(betta))
        print("gamma : %f" %(gamma))
        return alpha, betta, gamma

    def update_pedestrian_angle(self, pedestrian):
        if self.pedestrian_loc is None:
            self.pedestrian_loc = pedestrian.location
            return
        else:
            ped_direction = [pedestrian.location.y-self.pedestrian_loc.y,pedestrian.location.x-self.pedestrian_loc.x]
            lane_marker_direction = [self.lane_marker.y-self.lane_marker_prev.y, self.lane_marker.x-self.lane_marker_prev.x]
            x1,y1,x2,y2 = ped_direction[0],ped_direction[1],lane_marker_direction[0],lane_marker_direction[1]
            angle = np.arctan2(x1*y2-y1*x2,x1*x2+y1*y2) *180 /math.pi
            print("Angle between Pedestrian: ", angle)
            self.pedestrian_angle = angle
            return

    def get_ref_state(self, currState, obstacleList, lane_marker, waypoint):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        # self.reachEnd = waypoint.reachedFinal
        p_last = lane_marker.lane_markers_center.location[-1]
        p_last_prev = lane_marker.lane_markers_center.location[-2]
        # # waypoint_angle = (p_last.y - p_last_prev.y)/(p_last.x - p_last_prev.x)
        # velo_angle = (currState[2][1]/(currState[2][0]+1e-9))
        #
        # left_borders, right_borders = self.get_borders(lane_marker)
        # left_last, left_last_prev = left_borders[-1], left_borders[-2]
        # right_last, right_last_prev = right_borders[-1], right_borders[-2]
        # left_border_vector = [left_last[0] - left_last_prev[0], left_last[1] - left_last_prev[1]]
        # right_border_vector = [right_last[0] - right_last_prev[0], right_last[1] - right_last_prev[1]]
        # left_extended = [left_last[0] + left_border_vector[0]*20, left_last[1] + left_border_vector[1]*20]
        # right_extended = [right_last[0] + right_border_vector[0]*20, right_last[1] + right_border_vector[1]*20]

        self.lane_marker = p_last
        self.lane_marker_prev = p_last_prev
        self.lane_state = lane_marker.lane_state
        self.velocity_multiplier = 1
        # if self.rrt_counter %50==0:
        #
        #     path = self.rrt_main(currState, left_borders, right_borders, obstacleList, left_extended, right_extended)
        #     print(len(path))
        #     self.waypoints_rrt = []
        #     for wp in path[40:]:
        #         self.waypoints_rrt.append([wp[0], wp[1]])
        #     self.waypoints_rrt = np.array(self.waypoints_rrt)
        # _, steer_angle_rrt = planner.plan(currState[0][0], currState[0][1], currState[1][2], 2.82461887897713965, 0.90338203837889, self.waypoints_rrt)
        # steer_angle_rrt/=2
        # self.rrt_counter +=1
        #
        # print("Steering angle Pure Pursuit + RRT: ", steer_angle_rrt)

                # print("Lane_markers distance", self.calc_dist(lane_marker.lane_markers_center.location[-1], lane_marker.lane_markers_left.location[-1]),self.calc_dist(lane_marker.lane_markers_center.location[-1], lane_marker.lane_markers_right.location[-1]))
        #Lane distance = 2.5 units
        if not obstacleList:
            self.target_lane = 4
        self.change_lane = self.target_lane != self.lane_state
        if not self.target_x or not self.target_y:
            self.target_x = self.lane_marker.x
            self.target_y = self.lane_marker.y
        if self.reachEnd:
            return None
        # print("Reach end: ", self.reachEnd)

        curr_x = currState[0][0]
        curr_y = currState[0][1]

        # Check whether any obstacles are in the front of the vehicle
        obs_front = False
        obs_left = False
        obs_right = False
        front_dist = np.inf
        pedestrian_dist = np.inf
        obs_front_dist = np.inf
        pedestrian_check = False
        if obstacleList:
            for obs in obstacleList:
                if "pedestrian" in obs.obstacle_name:
                    self.update_pedestrian_angle(obs)
                    print("Pedestrian present")
                    dy = obs.location.y - curr_y
                    dx = obs.location.x - curr_x
                    pedestrian_dist = min(pedestrian_dist, np.sqrt(dy * dy + dx * dx))
                    pedestrian_check = True
                # for vertex in obs.vertices_locations:
                #     dy = vertex.vertex_location.y - curr_y
                #     dx = vertex.vertex_location.x - curr_x
                #     yaw = currState[1][2]
                #     rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
                #     ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx
                # #
                #     psi = np.arctan(ry / rx)
                # # x = np.sqrt(dy * dy + dx * dx)
                # # front_dist = min(front_dist, x)
                #     if rx > 0:
                #         front_dist = min(front_dist, np.sqrt(dy * dy + dx * dx))
                #         print("detected object is at {} away and {} radians".format(front_dist, psi))
                #         if psi < 0.2 and psi > -0.2:
                #             obs_front = True
                #         elif psi > 0.2:
                #             obs_right = True
                #         elif psi < -0.2:
                #             obs_left = True
                # alpha, beta, gamma = self.printAngle(obs.location, lane_marker.lane_markers_left.location[-1], lane_marker.lane_markers_right.location[-1])
                dy = obs.location.y - curr_y
                dx = obs.location.x - curr_x
                yaw = currState[1][2]

                rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
                x = np.sqrt(dy * dy + dx * dx)
                if x < self.detect_dist and rx>0:
                    front_dist = min(front_dist, x)

                    print("Obstacle in range: ", obs.obstacle_name)
                    alpha, beta, gamma = self.printAngle(obs.location, lane_marker.lane_markers_left.location[-1], lane_marker.lane_markers_right.location[-1])
                    if gamma > 90:
                        obs_right = True
                    elif beta > 90:
                        obs_left = True
                    else:
                        obs_front = True
                        obs_front_dist = min(x, obs_front_dist)
                else:
                    print("Obstacle not in range: ", obs.obstacle_name)
        print("Obs", obs_left, obs_front, obs_right)
        if not pedestrian_check:
            self.pedestrian_loc = None
        # self.obs_front_count +=1
        #         self.obs_front_count +=1
        #                 self.obs_front_count +=1
        if self.target_lane > self.lane_state:
            self.vehicle_state = "turn-right"
        elif self.target_lane < self.lane_state:
            self.vehicle_state = "turn-left"

        # prev_vehicle_state = self.vehicle_state
                ## Only for when one pedestrian!
        print("Pedestrian distance", pedestrian_dist)
        if pedestrian_dist < self.detect_dist/2 and len(obstacleList)==1 and not self.change_lane:
            print("Pedestrian Detected and planning to turn!!!", self.lane_state)
            if self.pedestrian_loc is not None and self.pedestrian_angle <0 and self.lane_state!=5:
                self.vehicle_state = "turn-right"
                self.target_lane +=1
                self.change_lane = True

            elif self.pedestrian_loc is not None and self.pedestrian_angle >0 and self.lane_state!=3:
                self.vehicle_state = "turn-left"
                self.target_lane -=1
                self.change_lane = True
            else:
                self.vehicle_state = "straight"
        else:
            if self.lane_state == LaneInfo.LEFT_LANE and not self.change_lane:
                if front_dist <= self.detect_dist and obs_front:
                    if not obs_right:
                        self.vehicle_state = "turn-right"
                        if front_dist < self.detect_dist/2:
                            self.change_lane = True
                            self.target_lane += 1
                    else:
                        self.vehicle_state = "stop"
                else:
                    self.vehicle_state = "straight"

            elif self.lane_state == LaneInfo.RIGHT_LANE and not self.change_lane:
                if front_dist <= self.detect_dist and obs_front:
                    if not obs_left:
                        self.vehicle_state = "turn-left"
                        if front_dist < self.detect_dist/2:
                            self.change_lane = True
                            self.target_lane -= 1
                    else:
                        self.vehicle_state = "stop"
                else:
                    self.vehicle_state = "straight"

            elif self.lane_state == LaneInfo.CENTER_LANE and not self.change_lane:
                if front_dist > self.detect_dist:
                    self.vehicle_state = "straight"
                else:
                    if not obs_front:
                        self.vehicle_state = "straight"
                    elif not obs_left:
                        self.vehicle_state = "turn-left"
                        if front_dist < self.detect_dist/2:
                            self.change_lane = True
                            self.target_lane -= 1
                    elif not obs_right:
                        self.vehicle_state = "turn-right"
                        if front_dist < self.detect_dist/2:
                            self.change_lane = True
                            self.target_lane += 1
                    else:
                        self.vehicle_state = "stop"

        obs_lane_counter = obs_left + obs_front + obs_right
        if obstacleList and len(obstacleList) < 3 and obs_lane_counter > 0:
            print("Less than 3 obstacles present")
            self.velocity_multiplier = min(0.99, self.velocity_multiplier)
        if obstacleList and len(obstacleList) > 2 and front_dist < self.detect_dist/2:
            print("3 or more obstacles present and very close!")
            self.velocity_multiplier = min(0.025, self.velocity_multiplier)
        if obstacleList and len(obstacleList) > 2 and front_dist > self.detect_dist/2 and obs_lane_counter==3:
            print("3 or more obstacles present but all paths are blocked")
            self.velocity_multiplier = min(0.025, self.velocity_multiplier)
        if front_dist < self.detect_dist/4 and obs_front:
            print("Obstacle very very close and in front of")
            self.velocity_multiplier = min(0.25, self.velocity_multiplier)
        if self.vehicle_state =="straight":
            print("Absolutely perfect")
            self.velocity_multiplier = min(1, self.velocity_multiplier)
        if front_dist > self.detect_dist/2 and self.target_lane == self.lane_state and self.vehicle_state !="straight":
            print("Slow down before ready to turn")
            self.velocity_multiplier = min(0.99, self.velocity_multiplier)
            self.vehicle_state = "straight"





        if self.vehicle_state == "stop":
            self.velocity_multiplier = 0.025
            self.speed = 0
        else:
            self.speed = 40
        if obs_front_dist < self.detect_dist/4 and obs_front and self.target_lane == self.lane_state:
            print("Stopping now!!!!!!")
            self.speed = -1
        if pedestrian_dist < np.inf:
            self.velocity_multiplier = min(0.025, self.velocity_multiplier)
            self.speed = min(0, self.speed)
        if pedestrian_dist < self.detect_dist/2:
            self.velocity_multiplier = min(0.025, self.velocity_multiplier)
        if obs_front_dist < self.detect_dist/7.5 and obs_front and self.target_lane == self.lane_state:
            print("Complete stopping now!!!!!!")
            self.speed = -2
        # if front_dist < self.detect_dist/2 and pedestrian_dist < self.detect_dist/4:
        #     self.vehicle_state = "stop"
        #     print("Stopping now!!!!!!")
        #     self.speed = -1



        print("Vehicle State: ", self.vehicle_state, "Vehicle Lane: ", self.lane_state,"Target Lane: ", self.target_lane, "Front distance", front_dist)

        # print(front_dist, self.lane_state, self.vehicle_state, obs_front, obs_left, obs_right)

        while not self.target_x or not self.target_y:
            continue

        distToTargetX = abs(self.target_x - curr_x)
        distToTargetY = abs(self.target_y - curr_y)

        if ((distToTargetX < 5 and distToTargetY < 5)):

            prev_target_x = self.target_x
            prev_target_y = self.target_y

            self.target_x = self.lane_marker.x
            self.target_y = self.lane_marker.y

            target_orientation = np.arctan2(self.target_y - prev_target_y,
                                            self.target_x - prev_target_x)

            if self.vehicle_state == "turn-right":
                # self.change_lane = False
                tmp_x = 4.5
                tmp_y = 0
                x_offset = np.cos(target_orientation + np.pi /
                                  2) * tmp_x - np.sin(target_orientation +
                                                      np.pi / 2) * tmp_y
                y_offset = np.sin(target_orientation + np.pi /
                                  2) * tmp_x + np.cos(target_orientation +
                                                      np.pi / 2) * tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset
            elif self.vehicle_state == "turn-left":
                # self.change_lane = False
                tmp_x = 4.5
                tmp_y = 0
                x_offset = np.cos(target_orientation - np.pi /
                                  2) * tmp_x - np.sin(target_orientation -
                                                      np.pi / 2) * tmp_y
                y_offset = np.sin(target_orientation - np.pi /
                                  2) * tmp_x + np.cos(target_orientation -
                                                      np.pi / 2) * tmp_y
                self.target_x = self.target_x + x_offset
                self.target_y = self.target_y + y_offset

        else:
            self.counter += 1
        return [self.target_x, self.target_y, self.speed, self.velocity_multiplier]


class VehicleController():
    def stop(self, delta=0):
        print("PRINTING STOP")
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = delta
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
        velocity_multiplier = targetPose[3]

        k_s = 0.1
        k_ds = 1
        k_n = 0.1
        k_theta = 1
        k_speed = 0.5

        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(
            currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        delta = k_n * yError
        min_v = max(10 * velocity_multiplier, 8)
        max_v = max(40 * velocity_multiplier, min_v + 1e-9)
        tmp_v = (min_v + 1/(k_speed*abs(yError) + 1/(max_v-min_v)))
        vError = tmp_v - curr_v
        # delta = min(delta, delta*(20/curr_v))
        # Checking if the vehicle need to stop
        # print("Length: ", len(lane_marker.lane_markers_center.location), len(lane_marker.lane_markers_center.location[:-10]))

        print("curr_v:", curr_v*3.6)
        if target_v > 0 or (curr_v < 8 and target_v > -1) or (curr_v < 3.5 and target_v != -2):
            if velocity_multiplier == 1:
                v = tmp_v
            else:
                v = xError * k_s + vError * k_ds
            print("velocity_multiplier: ", velocity_multiplier)
            print("Target_v: ", v)
            #Send computed control input to vehicle
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = v
            # newAckermannCmd.acceleration = 0
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd
        else:
            return self.stop(delta)


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
