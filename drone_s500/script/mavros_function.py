#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from pymavlink import mavutil

class MavrosFunction(object):
    def __init__(self):

        self.local_position = PoseStamped()
        self.state = State()
        self.extended_state = ExtendedState()

        self.sub_topics_ready = {key: False for key in ['local_pos', 'state', 'ext_state']}

        # ROS services
        service_timeout = 30
        rospy.loginfo("Waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # ROS subscribers
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                                              self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state', ExtendedState,
                                              self.extended_state_callback)

    #
    # Callback functions
    #
    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(self.state.mode, data.mode))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums['MAV_VTOL_STATE'][data.vtol_state].name))
        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums['MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        # old_arm = self.state.armed
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        # arm_set = False
        for i in range(timeout*loop_freq):
            if self.state.armed == arm:
                # arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        # old_mode = self.state.mode
        loop_freq = 1 # Hz
        rate = rospy.Rate(loop_freq)
        # mode_set = False
        for i in range(timeout*loop_freq):
            if self.state.mode == mode:
                # mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode) # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        # simulation_ready = False
        for i in range(timeout*loop_freq):
            if all(list(self.sub_topics_ready.values())):
                # simulation_ready = True
                rospy.loginfo("topics ready | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        # landed_state_confirmed = False
        for i in range(timeout*loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                # landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".format(i/loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo("waiting for VTOL transition | transition: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        # transitioned = False
        for i in range(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1}".format(i/loop_freq, timeout))
                # transitioned = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")
