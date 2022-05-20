# Custom controller - currently just follows the front-most center lane marker (with PD steering)

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

#import matplotlib.pyplot as plt
#from scipy.spatial import ConvexHull, convex_hull_plot_2d

from scipy import interpolate


class VehicleDecision():
    def __init__(self):
        self.target_x = None
        self.target_y = None
        self.target_theta = None

        self.detect_dist = 35
        self.speed = 30
        
        # get_ref_state args (for visualizer)
        self.currState = None
        # self.lane_state = 0     # for breaking
        self.obstacleList = None
        self.lane_marker = None
        self.waypoint = None

        self.center_center = None
        self.track_limit_left = None
        self.track_limit_right = None

        self.leftvertex = None
        self.rightvertex = None
        self.gap_vec = None


    def FGM(self,carpos,objects,left_track_limit,right_track_limit,carheading):

        phi1 = self.get_angle(carpos,carheading,left_track_limit)
        phi2 = self.get_angle(carpos,carheading,right_track_limit)

        if (len(objects)==0):
            leftvertex=left_track_limit
            rightvertex=right_track_limit
            gaparray = [phi2-phi1]
        else:

            objects.sort(key=lambda x: x[2][0])

            objects = np.array(objects)
            num_gaps = len(objects)+1
            gaparray = np.empty(num_gaps)
            gaparray[0] = objects[0,2,0]-phi1
            gaparray[num_gaps-1] = phi2-objects[num_gaps-2,2,1]

            for i in range(1,num_gaps-1):
                gaparray[i] = objects[i,2,0]-objects[i-1,2,1]

            # print("Gap array: ",gaparray*180/np.pi)

            maxgap = np.argmax(gaparray)

            if (maxgap==0):
                leftvertex = left_track_limit
                rightvertex = objects[0,0]
            elif (maxgap==num_gaps-1):
                leftvertex = objects[num_gaps-2,1]
                rightvertex = right_track_limit
            else:
                leftvertex = objects[maxgap-1,1]
                rightvertex = objects[maxgap,0]



        d2 = np.linalg.norm(leftvertex-carpos)
        d1 = np.linalg.norm(rightvertex-carpos)

        phi2 = np.abs(self.get_angle(carpos,carheading,leftvertex))
        phi1 = np.abs(self.get_angle(carpos,carheading,rightvertex))

        num = d1+(d2*np.cos(phi1+phi2))
        denom = np.sqrt(np.power(d1,2)+np.power(d2,2)+(2*d1*d2*np.cos(phi1+phi2)))
        phi_gap = np.arccos(num/denom)-phi1
        
        gap_vec = np.mean([leftvertex,rightvertex],axis=0)
        l = np.sqrt((d1**2+d2**2 - (2*d1*d2*np.cos(phi1+phi2))))

        # return (leftvertex,rightvertex,phi_gap,l,gap_vec,np.max(gaparray))
        return (leftvertex,rightvertex,phi_gap,l,gap_vec,len(objects),np.max(gaparray))


    def get_angle(self,carpos,yaw,objpos):
        curr_y = carpos[1]
        curr_x = carpos[0]          

        dy = objpos[1] - curr_y
        dx = objpos[0]  - curr_x
        rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
        ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx
        return np.arctan(ry / rx)


    def get_ref_state(self, currState, obstacleList, lane_marker, waypoint):

        LANE_LEFT = 3
        LANE_CENTER = 4
        LANE_RIGHT = 5

        self.currState = currState
        self.obstacleList = obstacleList
        self.lane_marker = lane_marker
        # self.lane_state = lane_marker.lane_state # For breaking
        self.waypoint = waypoint

        # TEMP DECISION MODULE

        self.lane_state = lane_marker.lane_state # Left (3), Center (4), Right (5)

        l = lane_marker.lane_markers_center.location[-1]
        r = lane_marker.lane_markers_center.rotation[-1]

        self.target_x = l.x
        self.target_y = l.y
        self.target_theta = r.y # This should be Z but by debugging, y is the only one that changes

        # \ TEMP DECISION MODULE

        carangle = currState[1][2]
        carpos = currState[0]

        # STEP 1: CALCULATE THE TRACK'S CENTER LINE AND LIMITS
        # We need this to determine if we're interpolating or not, it's also necessary if we do decide to interpolate

        lane_len = len(lane_marker.lane_markers_center.location)

        c = np.array([[lane_marker.lane_markers_center.location[i].x,lane_marker.lane_markers_center.location[i].y] for i in range(lane_len)])
        r = np.array([[lane_marker.lane_markers_right.location[i].x,lane_marker.lane_markers_right.location[i].y] for i in range(lane_len)])
        l = np.array([[lane_marker.lane_markers_left.location[i].x,lane_marker.lane_markers_left.location[i].y] for i in range(lane_len)])

        if self.lane_state == LANE_LEFT:
            self.track_limit_left = l
            vec = r - c
            self.center_center = r+vec
            self.track_limit_right = r+4*vec
        elif self.lane_state == LANE_RIGHT:
            self.track_limit_right = r
            vec = l - c
            self.center_center = l+vec
            self.track_limit_left = l+4*vec
        else:
            self.center_center = c
            vec = l - c
            self.track_limit_left = l+2*vec
            vec = r - c
            self.track_limit_right = r+2*vec
        avg_lane_width = np.mean(np.linalg.norm(vec,axis=1)) # Needed to find track limits


        # STEP 2a: DECIDE IF WE'RE INTERPOLATING OR NOT
        # If the waypoint is further from the car than the furthest center center point, then we can interpolate
        wpoint = np.array([[waypoint.location.x,waypoint.location.y]])
        cpoint = self.center_center[-1]

        wdist = np.linalg.norm(wpoint-carpos)
        cdist = np.linalg.norm(cpoint-carpos)

        # STEP 2b: INTERPOLATE
        if (wdist > cdist): # waypoint is further, interpolate
            tmp_left = self.track_limit_left
            tmp_cent = self.center_center
            tmp_right = self.track_limit_right

            # First need to interpolate center lane, then widen to get track limit interpolation points
    
            points = np.concatenate([tmp_cent,wpoint])

            x_points = points.T[0]
            y_points = points.T[1]
            tck,u = interpolate.splprep([x_points, y_points])
            unew = np.arange(0, 1.01, 0.01)
            out = interpolate.splev(unew, tck)

            self.center_center = np.array(out).T
            
            # Now get track limit points and interpolate them

            vec = self.center_center[-1]-self.center_center[-2]
            
            left_vec = np.array([vec[1],-vec[0]])
            right_vec = np.array([-vec[1],vec[0]])

            left_vec = 3*avg_lane_width*left_vec/np.linalg.norm(left_vec)
            right_vec = 3*avg_lane_width*right_vec/np.linalg.norm(right_vec)

            waypoint_left = [wpoint[0]+left_vec]
            waypoint_right = [wpoint[0]+right_vec]

            points = np.concatenate([tmp_left,waypoint_left])

            x_points = points.T[0]
            y_points = points.T[1]
            tck,u = interpolate.splprep([x_points, y_points])
            unew = np.arange(0, 1.01, 0.01)
            out = interpolate.splev(unew, tck)
            
            self.track_limit_left = np.array(out).T

            points = np.concatenate([tmp_right,waypoint_right])

            x_points = points.T[0]
            y_points = points.T[1]
            tck,u = interpolate.splprep([x_points, y_points])
            unew = np.arange(0, 1.01, 0.01)
            out = interpolate.splev(unew, tck)
            self.track_limit_right = np.array(out).T





        left_psi = self.get_angle(carpos,carangle,self.track_limit_left[-1])
        right_psi = self.get_angle(carpos,carangle,self.track_limit_right[-1])



        FGMobjects = []
        obs_dis = []
        obs_angle = []

        for o in obstacleList:
            loc = np.array([o.location.x,o.location.y])
            dy = loc[1] - carpos[1]
            dx = loc[0] - carpos[0]
            rx = np.cos(-carangle) * dx - np.sin(-carangle) * dy
            
            # Obs angle to car
            obsangle = np.abs(self.get_angle(carpos,carangle,loc))

            v = o.vertices_locations[:4]
            v = np.array([[a.vertex_location.x,a.vertex_location.y] for a in v])
            a = [self.get_angle(carpos,carangle,x) for x in v]
            minangle = np.argmin(a)
            maxangle = np.argmax(a)

            if np.linalg.norm(loc-np.array(carpos)) > self.detect_dist or rx < 0:
                continue
            if np.linalg.norm(loc-np.array(carpos)) > self.detect_dist or obsangle > np.pi/3 or rx < 0:
                continue
            object = [v[minangle],v[maxangle],[a[minangle],a[maxangle]]]
            FGMobjects.append(object)
            obs_dis.append(np.linalg.norm(carpos-loc))
            obs_angle.append(obsangle)
        if not obs_dis:
            closest_obs_dis = np.inf
        else:
            closest_obs_dis = np.min(np.array(obs_dis))
            closest_obs_angle = obs_angle[np.argmin(np.array(obs_dis))]
            print("closest_obs_dis ",closest_obs_dis)
            print("closest_obs_angle ",closest_obs_angle)



        # print(c)
        # print(c-carpos)
        # print(np.shape(c-carpos))
        # nn = np.argmin(c - carpos)
        # print(nn)
        nntheta = self.get_angle(carpos,carangle,c[1])
        # print(nntheta)

        x = self.FGM(carpos,FGMobjects,self.track_limit_left[-1],self.track_limit_right[-1],carangle)

        self.leftvertex = x[0]
        self.rightvertex = x[1]

        # print("Phi gap: ", x[2]*180/np.pi)
        
        self.target_theta = 0.65*x[2] + 0.35*nntheta

        l = x[3]
        self.gap_vec = x[4]
        # print("closest_obs_dis ",closest_obs_dis)
        # print("num obs: ", x[5])
        # if (x[5]>=3 and closest_obs_dis<16) or (x[6]*180/np.pi<15):
        #     self.speed = 5
        #     self.target_x = lane_marker.lane_markers_center.location[-1].x
        #     self.target_y = lane_marker.lane_markers_center.location[-1].y
        # if closest_obs_dis < 25 and closest_obs_angle < np.pi/18:
        #     # self.speed = 0
        #     self.speed = 15
        
        self.target_x = self.gap_vec[0]
        self.target_y = self.gap_vec[1]

        if not obs_dis: #or closest_obs_angle > np.pi/4:
            self.target_x = c[-1][0]
            self.target_y = c[-1][1]
        # if closest_obs_dis < 10  and closest_obs_angle > np.pi/4 and closest_obs_angle < np.pi*2/3:
        #     self.target_x = c[-1][0]
        #     self.target_y = c[-1][1]
        else:
            self.target_x = self.gap_vec[0]
            self.target_y = self.gap_vec[1]

        
        if closest_obs_dis < 35  and closest_obs_angle < np.pi/12:
            self.speed = 20
        if closest_obs_dis < 15  and closest_obs_angle < np.pi/9:
            self.speed = 5
        if closest_obs_dis < 10  and closest_obs_angle < np.pi/4:
            self.speed = 0
        elif closest_obs_dis < 10  and closest_obs_angle > np.pi/4 and closest_obs_angle < np.pi*2/3:
            self.speed = 5
        else:
            self.speed = 30


        # print(self.target_theta*180/np.pi)

        # target = np.array(carpos)+np.array(x[3])

        # self.target_x = target[0]
        # self.target_y = target[1]

            

        return [self.target_x, self.target_y, self.speed,self.target_theta]

class VehicleController():
    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -30
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

        curr_theta = currentEuler[2]*180/np.pi
        # print("\n")

        # print("curr_theta raw:", curr_theta)
        #print("Raw curr_theta",curr_theta)
        # if (curr_theta < 0):
        #     curr_theta += 360
        curr_theta = (curr_theta+360)%360
        # print("curr_theta Guess:",curr_theta)

        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]
        target_theta = -targetPose[3]

        #0.015,0.01 almost gets sub 50 but doesn't work in the final turn
        #k_n = 0.01
        #k_theta = 0.015


        # Will score 52 on scenario 1
        #speed = 30
        #detect_dist = 30
        #k_s = 0.1
        #k_ds = 1
        #k_n = 0.01
        #k_theta = 0.05
        

        # Almost sub 50 on speed 35, detect dist 40
        # k_s = 0.1
        # k_ds = 1
        # k_n = 0.01
        # k_theta = 0.05

        # k_nonholom = -0.01


        # Score: 46 on speed 35, detect dist 40 on track 1
        # k_s = 0.1
        # k_ds = 1
        # k_n = 0.011
        # k_theta = 0.05

        # k_nonholom = -0.03

        # Track 3 score: 336 on detect dist 35, speed 30, 65/35 theta_split, 
        # k_s = 0.1
        # k_ds = 1
        # k_n = 0.045
        # k_theta = 0.2
        # decelerate_when_steering = 0.15

        k_s = 0.1
        k_ds = 1
        k_n = 0.038
        k_theta = 0.25
        decelerate_when_steering = 15

        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(
            currentEuler[2]) + (target_y - curr_y) * np.sin(currentEuler[2])
        yError = -(target_x - curr_x) * np.sin(
            currentEuler[2]) + (target_y - curr_y) * np.cos(currentEuler[2])
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        
        thetaError = target_theta# - curr_theta

        p = (k_n * yError)
        d = (k_theta * thetaError)
        delta = p + d

        # print("P = ",p)
        # print("D = ",d)

        # print("raw thetaError: ", thetaError)

        thetaErrorGuesses = np.array([thetaError-360,thetaError,thetaError+360])
        a = np.abs(thetaErrorGuesses)
        thetaError = thetaErrorGuesses[np.argmin(a)]
        # print("thetaError Geuss: ", thetaError)      

        target_v = max(0,target_v - decelerate_when_steering*abs(delta))
        vError = target_v - curr_v
        print("vError ",vError)
        print("curr_v : ",curr_v)
        print("target_v :",target_v)
        # print("\n")
        # Checking if the vehicle need to stop
        if target_v > 0:
            newAckermannCmd = AckermannDrive()
            # v = xError * k_s + vError * k_ds + (k_nonholom*np.abs(thetaError))
            v = xError * k_s + vError * k_ds

            # amplify decceleration
            if vError < 0:
                # if curr_v>5:
                newAckermannCmd.acceleration = -30
                # v = 5
                print("=====Deccelerating======")
            # print("v:", v)
            # print("vError", vError)
            #Send computed control input to vehicle
            newAckermannCmd.speed = v
            newAckermannCmd.steering_angle = delta
            return newAckermannCmd
        else:
            return self.stop()

            # Send computed control input to vehicle
        # newAckermannCmd = AckermannDrive()
        # newAckermannCmd.speed = 20
        # newAckermannCmd.steering_angle = target_theta
        # return newAckermannCmd        


##class CustomVehiclePerception:
##    def __init__(self, role_name='ego_vehicle'):
##        rospy.Subscriber("/carla/%s/location" % role_name, LocationInfo,
##                         self.locationCallback)
##        rospy.Subscriber("/carla/%s/obstacles" % role_name, ObstacleList,
##                         self.obstacleCallback)
##        rospy.Subscriber("/carla/%s/lane_markers" % role_name, LaneInfo,
##                         self.lanemarkerCallback)
##        rospy.Subscriber("/carla/%s/waypoints" % role_name, WaypointInfo,
##                         self.waypointCallback)
##
##        self.position = None
##        self.rotation = None
##        self.velocity = None
##        self.obstacleList = None
##        self.lane_marker = None
##        self.waypoint = None
##
##    def locationCallback(self, data):
##        self.position = (data.location.x, data.location.y)
##        self.rotation = (np.radians(data.rotation.x),
##                         np.radians(data.rotation.y),
##                         np.radians(data.rotation.z))
##        self.velocity = (data.velocity.x, data.velocity.y)
##
##    def obstacleCallback(self, data):
##        self.obstacleList = data.obstacles
##
##    def lanemarkerCallback(self, data):
##        self.lane_marker = data
##
##    def waypointCallback(self, data):
##        self.waypoint = data
##
##    def ready(self):
##        return (self.position
##                is not None) and (self.rotation is not None) and (
##                    self.velocity
##                    is not None) and (self.obstacleList is not None) and (
##                        self.lane_marker is not None
##                    )  and (self.waypoint is not None)
##
##    def clear(self):
##        self.position = None
##        self.rotation = None
##        self.velocity = None
##        self.obstacleList = None
##        self.lane_marker = None
##        self.waypoint = None
##
##class CustomVisualizer():
##    def __init__(self):
##        plt.ion()
##        self.fig = plt.figure()
##        self.ax = plt.axes(xlim=(-60, 60), ylim=(-60, 60))
##        self.ego_pose = [np.zeros([2, 1]), 0.]
##        self.decisionModule = VehicleDecision()
##        self.controlModule = VehicleController()
##
##    def add_obs(self, obstacleList):
##        for ob in obstacleList:
##            center = np.array([ob.location.x, ob.location.y])
##            center = self.trans(center)
##            self.ax.plot(center[0], center[1], 'ro', linewidth=2.)
##            if len(ob.vertices_locations) > 0:
##                assert len(ob.vertices_locations) == 8
##                vertices = [v.vertex_location for v in ob.vertices_locations]
##                vertices = [[p.x, p.y, p.z] for p in vertices]
##                half_level = np.mean([v[2] for v in vertices])
##                vertices = np.array([v for v in vertices if v[2] > half_level])[:,:2].T
##                vertices = self.trans(vertices).T
##                hull = ConvexHull(vertices)
##                idxs = list(hull.vertices) + list(hull.vertices[0:1])
##                self.ax.plot(vertices[idxs, 0], vertices[idxs, 1], 'r-', lw=2)
##
##    def add_lane(self, lane_markers):
##        points = []
##        for lane in [lane_markers.lane_markers_center, lane_markers.lane_markers_left, lane_markers.lane_markers_right]:
##            points += lane.location
##        points = [[p.x, p.y, p.z] for p in points]
##        points = np.array(points).T[:2, :] # 2 x N
##        points = self.trans(points)
##        self.ax.plot(points[0,:], points[1,:], 'k*', linewidth=1.)
##
##    def add_self(self):
##        w = 1.5; h = 3;
##        self.ax.plot([-w/2, -w/2, w/2, w/2, -w/2], [-h/2, h/2, h/2, -h/2, -h/2], 'b-', linewidth=1.)
##
##    def custom(self,d):
##        if d.waypoint is not None:
##            loc = d.waypoint.location
##            loc = self.trans(np.array([loc.x,loc.y]))
##            self.ax.plot(loc[0],loc[1], 'go',ms=8)
##
##
##        if d.track_limit_left is not None:
##            for i in d.track_limit_left:
##                loc = self.trans(np.array([i[0],i[1]]))
##                self.ax.plot(loc[0],loc[1], 'o',color='blue',ms=4)               
##        if d.center_center is not None:
##            for i in d.center_center:
##                loc = self.trans(np.array([i[0],i[1]]))
##                self.ax.plot(loc[0],loc[1], 'o',color='purple',ms=4)
##        if d.track_limit_right is not None:
##            for i in d.track_limit_right:
##                loc = self.trans(np.array([i[0],i[1]]))
##                self.ax.plot(loc[0],loc[1], 'o',color='red',ms=4)
##
##        if d.leftvertex is not None:
##            loc = self.trans(np.array([d.leftvertex[0],d.leftvertex[1]]))
##            self.ax.plot(loc[0],loc[1], 'o',color='gold',ms=16)
##
##        if d.rightvertex is not None:
##            loc = self.trans(np.array([d.rightvertex[0],d.rightvertex[1]]))
##            self.ax.plot(loc[0],loc[1], 'o',color='lawngreen',ms=16)
##
##        if d.gap_vec is not None:
##            loc = self.trans(d.gap_vec)
##            self.ax.plot(loc[0],loc[1], 'o',color='deeppink',ms=16)
##        # loc = self.trans(d.track_limit_right)
##        # self.ax.plot(loc[0],loc[1], 'o',color='dodgerblue',ms=2)
##
##    def trans(self, points):
##        if len(points.shape) == 1:
##            p = points.reshape(-1, 1)
##        elif len(points.shape) == 2:
##            p = points
##        else:
##            raise ValueError('wrong shape')
##        assert p.shape[0] == 2
##
##        rotation = np.pi / 2 - self.ego_pose[1]
##        shift = self.ego_pose[0]
##        p = np.matmul(np.array([[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]]), p - shift)
##        p[0,:] = -p[0,:]
##        return p.reshape(points.shape)
##
##    def update(self, pm):
##        self.ax.clear()
##        self.ax.set_title('t = %.3f s'%rospy.get_time())
##        n = 30
##        self.ax.set_xlim(-n, n)
##        self.ax.set_ylim(-n, n)
##        self.ego_pose = [np.array(pm.position).reshape(-1, 1), pm.rotation[-1]]
##        self.add_self()
##        self.custom(self.decisionModule)
##        self.add_lane(pm.lane_marker)
##        self.add_obs(pm.obstacleList)
##        self.fig.canvas.draw()
##        self.fig.canvas.flush_events()

 class Controller(object):
     """docstring for Controller"""
     def __init__(self):
         super(Controller, self).__init__()
         role_name = rospy.get_param("~role_name", "ego_vehicle")
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


##class Controller(object):
##    """docstring for Controller"""
##    def __init__(self):
##        super(Controller, self).__init__()
##        role_name = rospy.get_param("~role_name", "ego_vehicle")
##        self.pm = CustomVehiclePerception(role_name=role_name) # Copied from graic_core/scripts/visualize_2D.py
##        self.vis = CustomVisualizer() # Copied and modified from graic_core/scripts/visualize_2D.py
##        self.decisionModule = self.vis.decisionModule
##        self.controlModule = self.vis.controlModule
##
##    def stop(self):
##        return self.controlModule.stop()
##
##    def execute(self, currState, obstacleList, lane_marker, waypoint):
##        # Get the target state from decision module
##        refState = self.decisionModule.get_ref_state(currState, obstacleList,
##                                                     lane_marker, waypoint)
##        if self.pm.ready():
##            self.vis.update(self.pm)
##        if not refState:
##            return None
##        return self.controlModule.execute(currState, refState)
