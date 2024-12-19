#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import json
from .joint_params import HEXARMJointCtlExtraParams, HEXARMJointStateMsg
from xpkg_arm_msgs.msg import XmsgArmJointParamList


class ArmController:
    def __init__(self, name: str = "unknown"):
        ### ros node
        rospy.init_node(name, anonymous=True)

        ### publisher
        self.pub = rospy.Publisher('joint_cmd', XmsgArmJointParamList, queue_size=10)

        ### data list
        self.motor_cnt = None
        self.mode = None
        self.position = None
        self.velocity = None
        self.effort = None
        self.extra_param = None

        self.publish_index = 0
        self.loop_cnt = None

    def publish_joint_state(self, motor_cnt:int, mode:list, position:list, velocity:list, effort:list, extra_param:list):
        try:
            joints = HEXARMJointStateMsg(motor_cnt, mode, position, velocity, effort, extra_param)
            joints_msg = joints.get_msg()
            self.pub.publish(joints_msg)

        except Exception as e:
            rospy.logerr(f"Failed to publish message[{self.publish_index}]: {e}")
    
    def shutdown(self):
        if hasattr(self, '__spin_thread') and self.__spin_thread.is_alive():
            self.__spin_thread.join()
        rospy.signal_shutdown("Shutting down node.")

    ### loop demo below
    def init_publish_list(self, motor_cnt:int, mode:list, position:list, velocity:list, effort:list, extra_param:list):
        if len(mode)!=len(position) or len(position)!=len(velocity) or len(velocity)!=len(effort) or len(effort)!=len(extra_param):
            rospy.logerr("The list length is not consistent, init puhlish list failed.")
            return
        else:
            self.loop_cnt = len(mode)
        self.motor_cnt = motor_cnt
        self.mode = mode
        self.position = position
        self.velocity = velocity
        self.effort = effort
        self.extra_param = extra_param

    def publish_loop(self, event=None):
        if self.loop_cnt == None:
            return
        else:
            self.publish_joint_state(self.motor_cnt, 
                                     self.mode[self.publish_index], 
                                     self.position[self.publish_index], 
                                     self.velocity[self.publish_index], 
                                     self.effort[self.publish_index], 
                                     self.extra_param[self.publish_index]
                                    )
            self.publish_index = (self.publish_index + 1) % self.loop_cnt

    def work(self, cycle: int):
        rospy.Timer(rospy.Duration(cycle), self.publish_loop)
        rospy.loginfo("Node is running... Press Ctrl+C to stop.")
        rospy.spin()