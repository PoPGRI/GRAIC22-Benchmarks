#!/usr/bin/env python3
import rospy
import rospkg
import numpy as np
import argparse
import time
import roslaunch
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo, WaypointInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo
from GoHeelsRacing.msg import LineSegment, VoronoiPlannerInput, VoronoiPlannerOutput
from geometry_msgs.msg import Vector3
import carla
import math


def Loc_to_V3(l):
    return Vector3(l.x, l.y, l.z)


def V3_to_Loc(v):
    # Vector3 to carla.Location
    return carla.Location(v.x, v.y, v.z)


def V3_to_Rot(v):
    # Vector3 to carla.Rotation
    return carla.Rotation(v.x, v.y, v.z)


class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.role_name = role_name
        self.subVehicleInfo = rospy.Subscriber(
            "/carla/%s/vehicle_info" % role_name, CarlaEgoVehicleInfo, self.vehicleInfoCallback)

        self.wheelbase = 2.0  # will be overridden by vehicleInfoCallback
        # will be overridden by vehicleInfoCallback
        self.max_steer_rad = math.radians(70)
        self.max_speed = 25
        self.min_speed = 10

        self.old_steering = 0.0 # new variable to tune the turning based on speed

        self.brake_coeff = 1.0  # To tune speed_diff-throttle curve



    def vehicleInfoCallback(self, data):
        p = [w.position for w in data.wheels]
        r2f_x = 0.5*(p[0].x + p[1].x - (p[2].x + p[3].x))
        r2f_y = 0.5*(p[0].y + p[1].y - (p[2].y + p[3].y))
        self.wheelbase = np.linalg.norm([r2f_x, r2f_y])
        print(f'Controller set wheelbase to {self.wheelbase} meters.')
        rospy.logwarn(f'Controller set wheelbase to {self.wheelbase} meters.')

        self.max_steer_rad = data.wheels[0].max_steer_angle
        print(f'Controller set max_steer_rad to {self.max_steer_rad} radians.')
        rospy.logwarn(f'Controller set max_steer_rad to {self.max_steer_rad} radians.')


    def initiate(self):
        self.carlaControlPub.publish(CarlaEgoVehicleControl())

    def stop(self):
        rospy.logwarn("stop")
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        return newAckermannCmd

    def execute(self, currentState, targetState):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """
        import math
        if not targetState[0] or not targetState[1]:
            self.carlaControlPub.publish(CarlaEgoVehicleControl())
            rospy.logwarn("no control")
            return

        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]

        currentSpeed = math.sqrt(currentState[2][0] ** 2 + currentState[2][1] ** 2)
        speed_error = targetState[2] - currentSpeed
        curr_speed = np.clip(currentSpeed, self.min_speed, self.max_speed)

        car_loc = carla.Location(curr_x, curr_y, 0)
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)
        car2map = carla.Transform(car_loc, car_rot)
        rear_axle = car2map.transform(
            carla.Location(-self.wheelbase/2.0, 0, 0))

        target_x = targetState[0]
        target_y = targetState[1]

        dx = target_x - curr_x
        dy = target_y - curr_y
        # Rotate (dx, dy) by -currentEuler[2] radians
        xError = dx*np.cos(currentEuler[2]) + dy*np.sin(currentEuler[2])
        yError = -dx*np.sin(currentEuler[2]) + dy*np.cos(currentEuler[2])

        # transform (xError, yError) to wheelbase coordinates
        xError += self.wheelbase/2.0

        # pure-pursuit steering rule
        import math
        d2 = xError**2 + yError**2
        steer_rad = math.atan(2 * self.wheelbase * yError / d2)

        # Map the steering angle to ratio of maximum possible steering angle
        steer_rad = np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad)
        steer_ratio = steer_rad/self.max_steer_rad
        accelerate_activation = 5.0
        brake_activation = -2.0
        coast_activation = 0.0
        if speed_error > coast_activation:
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.acceleration = 0  # Change speed as quickly as possible
            newAckermannCmd.speed = targetState[2]
            newAckermannCmd.steering_angle = steer_rad
            # Change angle as quickly as possible:
            newAckermannCmd.steering_angle_velocity = 5
            #self.ackermannControlPub.publish(newAckermannCmd)
            return newAckermannCmd
        elif speed_error >= brake_activation:
            newControlCmd = CarlaEgoVehicleControl()
            newControlCmd.throttle = 0
            newControlCmd.steer = steer_ratio
            newControlCmd.brake = 0
            newControlCmd.hand_brake = False
            newControlCmd.reverse = False
            newControlCmd.manual_gear_shift = False
            # self.carlaControlPub.publish(newControlCmd)
            return newControlCmd
        else:  # brake_activation > speed_error
            brake = min(1,
                        math.atan(self.brake_coeff*(brake_activation-speed_error)))
            newControlCmd = CarlaEgoVehicleControl()
            newControlCmd.throttle = 0
            newControlCmd.steer = steer_ratio
            newControlCmd.brake = brake
            newControlCmd.hand_brake = False
            newControlCmd.reverse = False
            newControlCmd.manual_gear_shift = False
            # self.carlaControlPub.publish(newControlCmd)
            return newControlCmd

class VehicleDecision():
    def __init__(self, role_name='ego_vehicle'):
        rospy.Subscriber("/carla/%s/vehicle_info" % role_name, CarlaEgoVehicleInfo, self.vehicleInfoCallback)
        self.voronoiPub = rospy.Publisher(
            "/planner_input", VoronoiPlannerInput, queue_size=1)
        self.subVoronoi = rospy.Subscriber(
            "/planner_output", VoronoiPlannerOutput, self.planCallback)


        self.position = None
        self.velocity = None
        self.rotation = None
        self.obstacleList = None

        self.lookahead = 5.0  # meters
        self.wheelbase = 2.0  # will be overridden by vehicleInfoCallback
        self.allowed_obs_dist = 1.7  # meters from Voronoi diagram to obstacles
        self.max_speed = 25
        self.min_speed = 10
        self.speed_coeff = 0.35  # to tune the speed controller
        self.friction_coeff = 1.3
        self.old_prediction = []
        self.pos_history = []

        self.plan = []
        self.roadmap = []
        self.reachEnd = False

        self.plan_update = 0

        self.milestone = None
        self.lane_info = None

        # start planner node
        self.planner_package = 'GoHeelsRacing'
        self.planner_executable = 'planner'
        self.planner_node = roslaunch.core.Node(self.planner_package, self.planner_executable)
        self.planner_launch = roslaunch.scriptapi.ROSLaunch()
        self.planner_launch.start()
        self.planner_process = self.planner_launch.launch(self.planner_node)
        print(self.planner_process.is_alive())

        # For debuggin purposes. TODO delete later
        # host = rospy.get_param('~host', 'localhost')
        # port = rospy.get_param('~port', 2000)
        # client = carla.Client(host, port)
        # self.world = client.get_world()


    def vehicleInfoCallback(self, data):
        p = [w.position for w in data.wheels]
        r2f_x = 0.5*(p[0].x + p[1].x - (p[2].x + p[3].x))
        r2f_y = 0.5*(p[0].y + p[1].y - (p[2].y + p[3].y))
        self.wheelbase = np.linalg.norm([r2f_x, r2f_y])
        print(f'Planner set wheelbase to {self.wheelbase} meters.')

    def planCallback(self, data):
        self.plan_update = 0
        self.plan = [self.rearAxle_to_map(
            self.currState, V3_to_Loc(v)) for v in data.plan]

        self.roadmap = [(self.rearAxle_to_map(self.currState, V3_to_Loc(seg.start)),
                         self.rearAxle_to_map(self.currState, V3_to_Loc(seg.end)))
                        for seg in data.roadmap]

        #---- Visualization ----
        # h0 = carla.Location(
        #     0, 0, self.lane_info.lane_markers_center.location[-1].z + 0.5)
        # h1 = carla.Location(
        #     0, 0, self.lane_info.lane_markers_center.location[-1].z + 1.0)
        # Roadmap
        # for loc0, loc1 in self.roadmap:
        #     self.world.debug.draw_line(
        #         loc0+h0, loc1+h0, thickness=0.1, color=carla.Color(255, 255, 255), life_time=0.1)
        # # Plan
        # for i in range(len(self.plan)-1):
        #     self.world.debug.draw_line(
        #         self.plan[i]+h1, self.plan[i+1]+h1, thickness=0.2, color=carla.Color(0, 255, 0), life_time=0.1)

    def rearAxle_to_map(self, currentState, loc):
        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]

        car_loc = carla.Location(curr_x, curr_y, 0)
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)
        car2map = carla.Transform(car_loc, car_rot)
        loc_in_car = loc - carla.Location(self.wheelbase/2., .0, .0)
        return carla.Location(car2map.transform(loc_in_car))

    def map_to_rearAxle(self, currentState, loc):
        currentEuler = currentState[1]
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)

        forward = car_rot.get_forward_vector()
        right = car_rot.get_right_vector()

        ra = self.rearAxle(currentState)
        v = loc - ra
        vx = forward.x * v.x + forward.y * v.y
        vy = right.x * v.x + right.y * v.y
        return carla.Location(vx, vy, 0)

    def rearAxle(self, currentState):  # TODO use map_to_rearAxle instead
        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]
        car_loc = carla.Location(curr_x, curr_y, 0)
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)
        car2map = carla.Transform(car_loc, car_rot)
        rear_axle = carla.Location(car2map.transform(
            carla.Location(-self.wheelbase/2.0, 0, 0)))
        return rear_axle

    def ccw(self, points):
        # Sort vertices of a convex polygon counter-clockwise
        ps = sorted(points, key=lambda p: p.x)
        pccw = []
        for p in ps[1:]:
            v = p-ps[0]
            l = math.sqrt(v.x**2+v.y**2)
            if (l < 0.001):  # Bicycles are thin
                continue
            sin = v.y/l
            pccw.append((sin, p))
        pccw.sort(key=lambda pair: pair[0])
        return [ps[0]] + [p[1] for p in pccw]

    def pubPlannerInput(self, currentState, obstacles):

        obstacles = self.obstacleList
        road_boundaries = []

        # If no milestones seen so far
        if not self.milestone:
            print('No milestones seen yet!')
            return
        if not self.lane_info:
            print('No lane info yet!')
            return

        right_border, left_border = [], []
        right_waypoints = self.lane_info.lane_markers_right
        center_waypoints = self.lane_info.lane_markers_center
        left_waypoints = self.lane_info.lane_markers_left
        for i, (loc, rot, center) in enumerate(zip(right_waypoints.location, right_waypoints.rotation, center_waypoints.location)):
            loc, right = V3_to_Loc(loc), V3_to_Rot(rot).get_right_vector()
            c = V3_to_Loc(center)
            lane_width = c.distance(loc)*2
            lane_count = abs(self.lane_info.RIGHT_LANE -
                             self.lane_info.lane_state)
            if c.distance(loc+right) < c.distance(loc):
                lane_count += 1
            border = loc + right*lane_width*lane_count + \
                right*0.8 + carla.Location(0, 0, 0.2)
            right_border.append(border)

        for loc, rot, center in zip(left_waypoints.location, left_waypoints.rotation, center_waypoints.location):
            loc, left = V3_to_Loc(loc), V3_to_Rot(rot).get_right_vector()*(-1)
            c = V3_to_Loc(center)
            lane_width = c.distance(loc)*2
            lane_count = abs(self.lane_info.LEFT_LANE -
                             self.lane_info.lane_state)
            if c.distance(loc+left) < c.distance(loc):
                lane_count += 1
            border = loc + left*lane_width*lane_count + \
                left*0.8 + carla.Location(0, 0, 0.2)
            left_border.append(border)

        road_center = left_border[-1]*0.5 + right_border[-1]*0.5

        # Linear extrapolation after the last boundary,
        # based on the direction from road center to milestone
        ext_size = 16  # meters
        center_to_milestone = self.milestone.distance(road_center)
        if center_to_milestone > 8.0:
            ext1 = (self.milestone - road_center) * ext_size \
                / center_to_milestone
        else:
            lane_center_rot = carla.Rotation(
                0, self.lane_info.lane_markers_center.rotation[-1].y, 0)
            ext1 = lane_center_rot.get_forward_vector()*ext_size

        road_boundaries += [(right_border[i], right_border[i+1])
                            for i in range(len(right_border)-1)]

        for i in range(6):
            road_boundaries += [(right_border[-1]+i*ext1/5.0, right_border[-1]+(i+1)*ext1/5.0)]

        road_boundaries += [(left_border[i], left_border[i+1])
                            for i in range(len(left_border)-1)]

        for i in range(6):
            road_boundaries += [(left_border[-1]+i*ext1/5.0, left_border[-1]+(i+1)*ext1/5.0)]

        milestone = road_center+ext1*1.5

        obstacle_boundaries = []
        for obs in obstacles:
            poly = []
            if len(obs.vertices_locations) == 0:
                continue
            for i in range(0, len(obs.vertices_locations), 2):
                vec = obs.vertices_locations[i].vertex_location
                poly.append(carla.Location(vec.x, vec.y, 0.0))
            poly = self.ccw(poly)
            obstacle_boundaries += [(poly[i-1], poly[i])
                                    for i in range(len(poly))]

        front = self.rearAxle_to_map(
            currentState, carla.Location(self.wheelbase*1.5, 0, 0))

        boundaries = road_boundaries + obstacle_boundaries

        # Publish
        data = VoronoiPlannerInput()
        data.allowed_obs_dist = self.allowed_obs_dist
        data.car_location = Loc_to_V3(
            self.map_to_rearAxle(currentState, front))
        data.milestone = Loc_to_V3(
            self.map_to_rearAxle(currentState, milestone))
        for segment in boundaries:
            s0 = self.map_to_rearAxle(currentState, segment[0])
            s1 = self.map_to_rearAxle(currentState, segment[1])
            start = Vector3(s0.x, s0.y, 0.0)
            end = Vector3(s1.x, s1.y, 0.0)
            data.obstacles.append(LineSegment(start, end))

        self.voronoiPub.publish(data)

        # ----- Visualization----
        # Road boundaries
        # for bound in road_boundaries:
        #     self.world.debug.draw_line(
        #         bound[0], bound[1], thickness=0.2, life_time=0.1)

        # Milestone
        # self.world.debug.draw_line(
        #     milestone, milestone+carla.Location(0, 0, 5), color=carla.Color(0, 255, 0), life_time=0.1)


    def map_to_line_segment(self, line_segment, loc, angle): # transform global location and angle to withrespect to the location frame of the line_segment
        origin = line_segment[0]
        line_angle = math.atan2(line_segment[1].y-line_segment[0].y, line_segment[1].x-line_segment[0].x)
        car_rot = carla.Rotation(0, math.degrees(line_angle), 0)
        forward = car_rot.get_forward_vector()
        right = car_rot.get_right_vector()
        v = loc - origin
        vx = forward.x * v.x + forward.y * v.y
        vy = right.x * v.x + right.y * v.y
        l = carla.Location(vx, vy, 0)
        a = angle - line_angle
        if a < -math.pi:
            a += 2*math.pi
        if a > math.pi:
            a -= 2*math.pi
        return (l, a)

    def line_segment_to_map(self, line_segment, loc, angle): # transform local location and angle with respect to a line_segment to global
        origin = line_segment[0]
        line_angle = math.atan2(line_segment[1].y - line_segment[0].y, line_segment[1].x - line_segment[0].x)
        car_loc = carla.Location(origin.x, origin.y, 0)
        car_rot = carla.Rotation(0, np.degrees(line_angle), 0)
        car2map = carla.Transform(car_loc, car_rot)
        l = carla.Location(car2map.transform(loc))
        a = angle + line_angle
        if a < -math.pi:
            a += 2 * math.pi
        if a > math.pi:
            a -= 2 * math.pi
        return (l, a)

    def define_circle(self, p1, p2, p3):
        """
        Returns the radius of the circle passing the given 3 points.
        """
        temp = p2.x * p2.x + p2.y * p2.y
        bc = (p1.x * p1.x + p1.y * p1.y - temp) / 2
        cd = (temp - p3.x * p3.x - p3.y * p3.y) / 2
        det = (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y)

        if abs(det) < 1.0e-6: # the points form a line
            return np.inf

        # Center of circle
        cx = (bc*(p2.y - p3.y) - cd*(p1.y - p2.y)) / det
        cy = ((p1.x - p2.x) * cd - (p2.x - p3.x) * bc) / det

        radius = np.sqrt((cx - p1.x)**2 + (cy - p1.y)**2)
        return radius


    def get_min_turn_radius(self, pp): # use line segments to find the circle approximation of the path and determine max curv:
        predicted_pos = [pp[0]]
        for i in range(1, len(pp)):
            if pp[i].distance(predicted_pos[-1])<=0.1:
                continue
            predicted_pos.append(pp[i])
        min_turn_radius = math.inf
        min_turn_point = None
        for i in range(len(predicted_pos)-2):
            predicted_pos[i].z = 0.0
            predicted_pos[i+1].z = 0.0
            predicted_pos[i+2].z = 0.0
            turn_radius = self.define_circle(predicted_pos[i], predicted_pos[i+1], predicted_pos[i+2])
            if turn_radius < min_turn_radius:
                min_turn_radius = turn_radius
                min_turn_point = predicted_pos[i]

        #visualization
        # if min_turn_point != None:
        #     self.world.debug.draw_line(carla.Location(min_turn_point.x, min_turn_point.y, 0.0), carla.Location(min_turn_point.x, min_turn_point.y, 2.0), color=carla.Color(255, 0, 0), life_time=0.1)

        return min_turn_radius

    def update_friction_coeffecient(self, currentState):
        min_deviation_point = None
        current_speed_square = currentState[2][0] ** 2 + currentState[2][1] ** 2
        self.pos_history.append(self.rearAxle(currentState))
        # print("position history len: ", len(self.pos_history))
        if len(self.pos_history) >= 3:  # enough data point to estimate actual vehicle path's curvature
            # also need to check if we are moving forward respect to the plan
            path_radius = np.clip(self.define_circle(self.pos_history[-1], self.pos_history[-2], self.pos_history[-3]), 1e-5, 1e5)
            min_deviation_from_plan = None
            for i in range(len(self.old_prediction)):  # check distance between current position and old predicton
                deviation_from_plan = max(math.sqrt((self.pos_history[-1].x - self.old_prediction[i].x) ** 2 + (
                        self.pos_history[-1].y - self.old_prediction[i].y) ** 2), 1e-6)
                if min_deviation_from_plan == None or deviation_from_plan < min_deviation_from_plan:
                    min_deviation_from_plan = deviation_from_plan
                    min_deviation_point = self.old_prediction[i]
            # adjusting the frictional coeffecient
            if min_deviation_from_plan and self.pos_history[-1].distance(self.pos_history[-2]) > 0.1 and self.pos_history[-2].distance(self.pos_history[-3]) > 0.1:
                if min_deviation_from_plan < 0.075:
                    self.friction_coeff = min(max(self.friction_coeff, current_speed_square / (path_radius * 9.81)), 1.5)
                if min_deviation_from_plan > 0.075:
                    self.friction_coeff = max(min(self.friction_coeff, current_speed_square / (path_radius * 9.81)), 0.75)
            self.pos_history.pop(0)
        self.old_prediction = [] #reset old_prediction


    def get_speed(self, plan_full, currentState, obstacle_list, lookahead):  # use model predictive control to calculate target speed
        plan = [plan_full[0]]
        for p in plan_full:
            if p.distance(plan[-1]) < 0.5:  # track plan with line segments of 0.5m
                continue
            plan.append(p)
        nodes = []  # discrete line segments to be tracked node with line segments to be tracked
        for i in range(len(plan) - 1):
            nodes += [(plan[i], plan[i + 1])]

        obs_info = [] # we only need obstacle position and clearance for safety calculation
        for obs in obstacle_list:
            poly = []
            if len(obs.vertices_locations) == 0:
                continue
            for i in range(0, len(obs.vertices_locations), 2):
                vec = obs.vertices_locations[i].vertex_location
                poly.append(carla.Location(vec.x, vec.y, 0.0))
            obs_loc = carla.Location(obs.location.x, obs.location.y, 0.0)
            clearance = max([obs_loc.distance(p) for p in poly]) # the clearance is the max distance betweent position and vertice
            if len(obs.obstacle_name) < 7 or obs.obstacle_name[:7] != 'vehicle':
                # give pedestrain more clearance
                clearance = max(clearance, 2.0)
            obs_info += [(obs_loc, clearance)]

        current_speed_square = currentState[2][0] ** 2 + currentState[2][1] ** 2
        safety_margin = max(0.1 * current_speed_square, 10.0)

        global_loc = self.rearAxle(currentState)
        global_angle = currentState[1][2]
        predicted_pos = [global_loc]

        current_node_ind = 0
        step_size = 0.1  # step size = 0.2s
        total_pred_len = 0.0
        min_obs_dist = math.inf
        min_obs_pt = None

        while current_node_ind < len(nodes):
            current_node = nodes[current_node_ind]  # move on to the next line segment
            (relative_loc, relative_angle) = self.map_to_line_segment(current_node, global_loc, global_angle)
            line_segment_length = current_node[0].distance(current_node[1])
            # while math.sqrt(relative_loc.x**2 + relative_loc.y**2) < line_segment_length and relative_loc.x < line_segment_length:
            while math.sqrt((line_segment_length - relative_loc.x) ** 2 + relative_loc.y ** 2) >= lookahead and total_pred_len <= 13.0:
                angle_rate = 2 * (-math.sqrt(lookahead ** 2 - min(relative_loc.y ** 2, lookahead ** 2)) * math.sin(
                    relative_angle) - relative_loc.y * math.cos(relative_angle)) / lookahead ** 2
                relative_loc = carla.Location(relative_loc.x + step_size * math.cos(relative_angle),
                                              relative_loc.y + step_size * math.sin(relative_angle), 0.0)
                relative_angle = relative_angle + angle_rate * step_size
                if relative_angle > math.pi:
                    relative_angle -= 2 * math.pi
                if relative_angle < -math.pi:
                    relative_angle += 2 * math.pi  # record each step of numerical integration for testing purpose
                (temp_loc, temp_ang) = self.line_segment_to_map(current_node, carla.Location(relative_loc), relative_angle)
                total_pred_len += V3_to_Loc(predicted_pos[-1]).distance(V3_to_Loc(temp_loc))
                predicted_pos.append(temp_loc)

                if total_pred_len <= safety_margin:
                    car_loc = carla.Location(temp_loc.x, temp_loc.y, 0)
                    car_rot = carla.Rotation(0, np.degrees(temp_ang), 0)
                    car2map = carla.Transform(car_loc, car_rot)
                    loc_in_car = carla.Location(self.wheelbase/2., .0, .0)
                    ego_loc = carla.Location(car2map.transform(loc_in_car))
                    ego_clearance = self.wheelbase*0.75
                    for (obs_loc, obs_clearance) in obs_info:
                        if ego_loc.distance(obs_loc) < min_obs_dist:
                            min_obs_pt = ego_loc
                            min_obs_dist = ego_loc.distance(obs_loc)
                        if ego_loc.distance(obs_loc) < ego_clearance + obs_clearance:
                            self.old_prediction = predicted_pos
                            return 0.0

                if total_pred_len >= 13.0:
                    break
            (global_loc, global_angle) = self.line_segment_to_map(current_node, relative_loc, relative_angle)
            current_node_ind += 1
            if total_pred_len >= 13.0:
                break
        self.old_prediction = predicted_pos

        # visualization for collision avoidence
        # if min_obs_pt:
        #     print("min_obs_dist: ", min_obs_dist)
        #     min_obs_pt.z = 1.5
        #     self.world.debug.draw_point(
        #         min_obs_pt, 0.3,
        #         color=carla.Color(255, 0, 0), life_time=0.1)

        min_turn_radius = np.clip(self.get_min_turn_radius(predicted_pos), 1e-5, 1e5)
        speed = np.clip(math.sqrt(self.friction_coeff * min_turn_radius * 9.81), self.min_speed, self.max_speed)

        # visualization for rear axel movement prediction
        # for i in range(len(predicted_pos) - 1):
        #     self.world.debug.draw_line(
        #         carla.Location(predicted_pos[i].x, predicted_pos[i].y, 1.5),
        #         carla.Location(predicted_pos[i + 1].x, predicted_pos[i + 1].y, 1.5),
        #         color=carla.Color(0, 0, 255), life_time=0.1)

        return speed

    def updateState(self, position, rotation, velocity, obstacleList, lane_info, milestone):
        self.position = position
        self.rotation = rotation
        self.velocity = velocity
        self.obstacleList = obstacleList
        self.lane_info = lane_info
        self.milestone = carla.Location(
            milestone.location.x, milestone.location.y, milestone.location.z)
        self.reachEnd = milestone.reachedFinal
        self.currState = [position, rotation, velocity]

    def get_ref_state(self): # use constant lookahead distance so it's consistent with the model predictive speed control
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currentState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        if self.reachEnd:
            self.planner_process.stop()
            return None

        currentState = [self.position, self.rotation, self.velocity]
        obstacleList = self.obstacleList

        # record when was plan updated
        self.plan_update = min(40, self.plan_update + 1) # keep track of when plan was updated

        # Publish obstacles for VoronoiPlanner
        self.pubPlannerInput(
            currentState, obstacleList if obstacleList != None else [])

        #update friction coeffecient
        self.update_friction_coeffecient(currentState)

        # check if any obstacle is in close approximity
        no_obs_close = True
        for obs in obstacleList:
            if len(obs.vertices_locations) == 0:
                continue
            obs_loc = carla.Location(obs.location.x, obs.location.y, 0.0)
            ego_loc = carla.Location(currentState[0][0], currentState[0][1], 0.0)
            if ego_loc.distance(obs_loc) < 15.0:
                no_obs_close = False
                break

        if self.plan_update > 5:
            # plan hasn't been updated recently, only move forward when no obstacle is in close approximity
            if self.plan_update > 20:
                # planner cannot converge, restart
                self.planner_process.stop()
                self.planner_process = self.planner_launch.launch(self.planner_node)
                print(self.planner_process.is_alive())
                self.plan_update = 0
            if no_obs_close:
                return [self.lane_info.lane_markers_center.location[-1].x, self.lane_info.lane_markers_center.location[-1].y, 5.0]
            else:
                return [self.lane_info.lane_markers_center.location[-1].x, self.lane_info.lane_markers_center.location[-1].y, 0.0]

        if len(self.plan) < 5:
            # cannot find plan, stop immediately
            return [self.lane_info.lane_markers_center.location[-1].x, self.lane_info.lane_markers_center.location[-1].y, 0.0]


        # Find the waypoint with lookahead distance from real axle
        ra = self.rearAxle(currentState)

        if ra.distance(self.plan[-1]) <= self.lookahead:
            # cannot make prediction based on pure pursuit model
            # move forward no obstacle in close approximity
            if no_obs_close:
                return [self.plan[-1].x, self.plan[-1].y, 5.0]
            else:
                return [self.plan[-1].x, self.plan[-1].y, 0.0]

        # pad plan to make it predictable
        if ra.distance(self.plan[0]) >= self.lookahead:
            rospy.logwarn("plan does not start within lookahead circle")
            self.plan = [self.rearAxle_to_map(currentState, carla.Location(self.wheelbase*1.6, 0, 0))] + self.plan

        # calculate target speed
        speed = self.get_speed(self.plan, currentState, obstacleList, self.lookahead)

        # find target point
        for i, loc in enumerate(self.plan):
            if (loc.x - ra.x) ** 2 + (loc.y - ra.y) ** 2 > self.lookahead ** 2:
                break
        p_in = self.map_to_rearAxle(currentState, self.plan[i - 1])
        p_out = self.map_to_rearAxle(currentState, self.plan[i])

        # Intercept the lookahead circle with the plan segment (p_in, p_out)
        x1 = p_in.x
        y1 = p_in.y
        x2 = p_out.x
        y2 = p_out.y
        dx = x2 - x1
        dy = y2 - y1
        A = dx * dx + dy * dy
        B = x1 * dx + y1 * dy
        C = x1 * x1 + y1 * y1 - self.lookahead ** 2
        t = (-B + math.sqrt(max(B * B - A * C, 0.0))) / math.copysign(max(abs(A), 1e-5), A)

        target_in_rearAxle = carla.Location(x1 + t * dx, y1 + t * dy, 0)
        target = self.rearAxle_to_map(currentState, target_in_rearAxle)

        return [target.x, target.y, speed]

class Controller(object):
    """docstring for Controller"""
    def __init__(self, role_name='ego_vehicle'):
        super(Controller, self).__init__()
        self.decisionModule = VehicleDecision(role_name)
        self.controlModule = VehicleController()


    def stop(self):
        return self.controlModule.stop()

    def execute(self, currState, obstacleList, lane_marker, waypoint):
        # Get the target state from decision module
        self.decisionModule.updateState(currState[0], currState[1], currState[2], obstacleList, lane_marker, waypoint)
        refState = self.decisionModule.get_ref_state()
        if not refState:
            return None
        return self.controlModule.execute(currState, refState)
