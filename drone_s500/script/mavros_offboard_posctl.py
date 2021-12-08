#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_function import MavrosFunction
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread

class MavrosOffboardPosctl(MavrosFunction):
    """
    Tests flying in swing motion at a certain height in offboard control
    by sending  raw setpoints with appropriate type mask (z, vx, vy) via MAVROS.
    For the test to be successful it needs to reach the target height in a certain time.
    """

    def __init__(self):

	rospy.init_node('offboard_control_node')
        
	super(MavrosOffboardPosctl, self).__init__()
	
        self.raw_point = PositionTarget()
        self.radius = 0.15

        self.raw_setpoint_pb = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size =10)

        # send raw setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        #
        # Helper method
        #
    def send_pos(self):
        rate = rospy.Rate(20) #Hz
        self.raw_point.header = Header()
        self.raw_point.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.raw_point.header.stamp = rospy.Time.now()
            self.raw_point.coordinate_frame = 8
            self.raw_point.type_mask = 3064   #1475

            self.raw_setpoint_pb.publish(self.raw_point)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".
                       format(self.local_position.pose.position.x, self.local_position.pose.position.y,
                              self.local_position.pose.position.z))

        target_pos = np.array((x, y, z))
        current_pos = np.array((self.local_position.pose.position.x,
                                self.local_position.pose.position.y,
                                self.local_position.pose.position.z))
        return np.linalg.norm(target_pos - current_pos) < offset

    def is_at_height(self, h, offset):
        """offset: meters"""
        rospy.logdebug("current height | h:{0:.2f}".format(self.local_position.pose.position.z))

        return h - self.local_position.pose.position.z < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.raw_point.position.x = x
        self.raw_point.position.y = y
        self.raw_point.position.z = z
        rospy.loginfo("attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
                      format(x, y, z, self.local_position.pose.position.x,
                             self.local_position.pose.position.y,
                             self.local_position.pose.position.z))
        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        # reached = False
        for i in range(timeout*loop_freq):
            if self.is_at_position(self.raw_point.position.x,
                                   self.raw_point.position.y,
                                   self.raw_point.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(i/loop_freq, timeout))
                # reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def swing_motion(self, vx, h, timeout):
        """timeout(int): seconds"""
        rospy.loginfo("producing swing motion with velocity, at height | vx: {0:.2f}, h: {1:.2f}".format(vx, h))
        loop_freq = 0.8
        rate = rospy.Rate(loop_freq)
        for i in range(int(timeout*loop_freq)):
            self.raw_point.position.z = h
            self.raw_point.velocity.x = vx
            vx = -vx

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        self.raw_point.velocity.x = 0

    def test_swing_motion(self):
        """Test swing motion at a certain height"""

        # make sure topics are ready
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
	positions = [[0, 0, 1.7], [1, 0, 1.7], [0, 0, 1.7]]
        # positions = [[0, 0, 1.7]]
        # self.swing_motion(0., 2.0, 10)
	for position in positions:
	    self.reach_position(position[0], position[1], position[2], 30)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    swing_act = MavrosOffboardPosctl()
    swing_act.test_swing_motion()

