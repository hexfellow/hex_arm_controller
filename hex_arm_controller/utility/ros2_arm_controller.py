#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
import rclpy.node
import json
from .joint_params import HEXARMJointCtlExtraParams, HEXARMJointStateMsg
from xpkg_arm_msgs.msg import XmsgArmJointParamList


class ArmController:
    def __init__(self, name: str = "unknown", rate: int = 10):
        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__rate = self.__node.create_rate(rate)

        ### publisher
        self.pub = self.__node.create_publisher(XmsgArmJointParamList, 'joint_cmd', 10)

        self.current_msg_index = 0
        self.test_length = 3

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
            self.__logger.info(f"Failed to publish message[{self.publish_index}]: {e}")


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
        self.timer = self.__node.create_timer(cycle, self.publish_loop)
        self.__logger.info("Node is running... Press Ctrl+C to stop.")
        rclpy.spin(self.__node)



    def ok(self):
        return rclpy.ok()

    def sleep(self):
        self.__rate.sleep()

    def shutdown(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

    def __spin(self):
        rclpy.spin(self.__node)