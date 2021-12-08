#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from mavros_function import MavrosFunction
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

class MavrosOffboardPosctl(MavrosFunction):
    """
    Tests flying in swing motion at a certain height in offboard control
    by sending  raw setpoints with appropriate type mask (z, vx, vy) via MAVROS.
    For the test to be successful it needs to reach the target height in a certain time.
    """

    def __init__(self):

	rospy.init_node('swing_action_node')
        
	super(MavrosOffboardPosctl, self).__init__()
	
        self.pos = PoseStamped()
        self.radius = 0.09 # 
	self.radius_yz = 0.18 # collision region on yz plane
	self.cls_thresh = 0.12 # collision threshold; 0.124 rigid propeller

        self.pos_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)

        # send raw setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    #
    # Helper method
    #
    def send_pos(self):
        rate = rospy.Rate(20) #Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
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

    def is_collision(self, x, y, z, offset, epsilon):
	"""offset: meters, epsilon: meters"""

        rospy.logdebug("current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".
                       format(self.local_position.pose.position.x, self.local_position.pose.position.y,
                              self.local_position.pose.position.z))

	xerr = abs(self.local_position.pose.position.x - x)
	target_yz = np.array((y, z))
	feedback_yz = np.array((self.local_position.pose.position.y, self.local_position.pose.position.z))
	rospy.logwarn("yz_radius:{0:.2f}, x_error:{1:.2f}".format(np.linalg.norm(target_yz-feedback_yz), xerr))
	return np.linalg.norm(target_yz-feedback_yz) < offset and xerr < epsilon 

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = float(x)
        self.pos.pose.position.y = float(y)
        self.pos.pose.position.z = float(z)
        rospy.loginfo("Attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
                      format(x, y, z, self.local_position.pose.position.x,
                             self.local_position.pose.position.y,
                             self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in range(timeout*loop_freq):
            if self.is_at_position(x, y, z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(i/loop_freq, timeout))
                reached = True
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
	if not reached:
	    rospy.logwarn("[TIMEOUT] Fail to reach position | x: {0}, y: {1}, z: {2}".format(x, y, z))	

    def to_collision(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo("Towards collision at | x: {0}, y: {1}, z: {2}".format(x, y, z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does a collision occur in 'timeout' seconds?
        loop_freq = 50  # Hz
        rate = rospy.Rate(loop_freq)
        collision = False
        for i in range(timeout*loop_freq):
            if self.is_collision(x, y, z, self.radius_yz, self.cls_thresh):
                rospy.loginfo("collision occured | seconds: {0} of {1}".format(i/loop_freq, timeout))
                collision = True
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
	if not collision:
	    rospy.logwarn("[TIMEOUT] Experiment Failed")	
	
    def test_collision_response(self):
        """Test drone's response against a collision motion"""

        # make sure topics are ready
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

	positions = [[0, 0, 1.7], [1, 0, 1.7], [0, 0, 1.7]]
        # positions = [[0, 0, 1.7]]

	for position in positions:
	    self.reach_position(position[0], position[1], position[2], 30)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    swing_act = MavrosOffboardPosctl()
    swing_act.test_collision_response()

