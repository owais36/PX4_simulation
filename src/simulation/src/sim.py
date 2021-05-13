#!/usr/bin/env python
import rospy
import mavros
import time
from mavros_msgs.msg import State,PositionTarget
from mavros_msgs.srv import SetMode,CommandBool,CommandTOL
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import numpy as np


class UAV:
    def __init__(self):
        #Initializing the node here
        rospy.init_node("UAV_control")

        #Waiting for PX4 simulation to start successfully
        data = rospy.wait_for_message("/mavros/state",State)
        
        #Publsiher handle for UAV mode
        self.uav_mode_pub = rospy.Publisher("/UAV_mode", String,queue_size=10)

        #Publisher for sending position commands to UAV
        self.uav_pos_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        
        #Subscribing to topic /mavros/state here
        rospy.Subscriber("/mavros/state", State,self.callback_uav_state)

        #Subscribing to GPS position here
        rospy.Subscriber("/mavros/global_position/global", NavSatFix,self.callback_gps)

        #Subscribing to UAV local position here
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped,self.callback_local_pos)

        self.uav_state = State()
        self.gps_pos = NavSatFix()

        self.uav_position_local = PoseStamped()


    def main(self):
        #Check for UAV arming status and arming action
        self.set_uav_mode('AUTO.LOITER')
        if self.uav_state.armed == False:
            self.arm_uav()

        #Takeoff for the UAV
        self.takeoff(5)
        time.sleep(3)

        #Flying a circular trajectory
        self.uav_circle()

        #Landing
        self.set_uav_mode('AUTO.RTL')



    def callback_gps(self,msg):
        self.gps_pos.latitude = msg.latitude
        self.gps_pos.longitude = msg.longitude
        self.gps_pos.altitude = msg.altitude

    def callback_uav_state(self,msg):
        self.uav_state = msg
        self.uav_mode_pub.publish(msg.mode)

    def callback_local_pos(self,msg):
        self.uav_position_local = msg

    def arm_uav(self):
        arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        resp = arming_srv(True)

    def takeoff(self,alt):
        takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        resp = takeoff_srv(altitude=alt, latitude=self.gps_pos.latitude, longitude=self.gps_pos.longitude, min_pitch=0, yaw=0)
    
    def set_uav_mode(self,mode):
        setmode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        resp = setmode_srv(custom_mode = mode)

    def uav_circle(self):
        #Check UAV arming status first
        if self.uav_state.armed == False:
            self.arm_uav()
        
        #Generate a circular path at the height of 5 m
        path = []
        theta = np.linspace(0 , 2*np.pi,num=10,endpoint = True)
        for angle in theta:
            x = 5*np.sin(angle)
            y = 5*np.cos(angle)
            z = 5
            path.append((x,y,z))

        self.uav_offboard_trajectory(path)
    
    def uav_offboard_trajectory(self,path):
        rate = rospy.Rate(20.0)
        set_point = PositionTarget()
        set_point.type_mask = int('010111111000', 2)
        set_point.coordinate_frame = 1
        set_point.position.x = path[0][0]
        set_point.position.y = path[0][1]
        set_point.position.z = path[0][2]

        #Sending the First set point before enabling offboard mode
        for k in range(20):
            self.uav_pos_pub.publish(set_point)
        self.set_uav_mode('OFFBOARD')

        for p in path:
            dist_error = self.get_distance_error(p)
            set_point.position.x = p[0]
            set_point.position.y = p[1]
            set_point.position.z = p[2]
            while(dist_error > 0.2):
                self.uav_pos_pub.publish(set_point)
                rate.sleep()
                dist_error = self.get_distance_error(p)

    def get_distance_error(self,point):
        curr_x = self.uav_position_local.pose.position.x
        curr_y = self.uav_position_local.pose.position.y
        curr_z = self.uav_position_local.pose.position.z

        dist = np.sqrt((point[0]-curr_x)**2 + (point[1]-curr_y)**2 + (point[2]-curr_z)**2)
        return dist


if __name__ == '__main__':
    try:
        iris = UAV()
        iris.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Exiting!')