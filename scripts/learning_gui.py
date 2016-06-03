#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 03/06/16
@author: sampfeiffer
GUI to learn by demonstration for TIAGo
"""

import rospy
import rospkg
from sensor_msgs.msg import JointState
from play_motion_msgs.msg import PlayMotionActionGoal
from copy import deepcopy
import sys
import signal
from PyQt4 import QtGui, uic
from PyQt4.QtCore import QTimer


HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

#TODO: Take sliders stuff from https://github.com/pal-robotics/reem_movements_creator/blob/master/scripts/joints_sliders.py
#TODO: link buttons to their actions
#TODO: manage the gravity comp mode where we can command the wrist
#TODO: manage putting in gray all the things that cannot be used (non clickable)
#TODO: Implement the learning part as a node that accepts service calls for the start and stopping
#TODO: take from https://github.com/awesomebytes/python_qt_tutorial the examples for saving and showing pop up errors

def get_joint_val(joint_name, joint_states):
    for j_name in joint_states.name:
        if j_name == joint_name:
            j_idx = joint_states.name.index(j_name)
            j_val = joint_states.position[j_idx]
            return j_val


def load_params_from_yaml(complete_file_path):
    from rosparam import upload_params
    from yaml import load
    f = open(complete_file_path, 'r')
    yamlfile = load(f)
    f.close()
    upload_params('/', yamlfile)


class myGUI(QtGui.QMainWindow):

    joint_name_list = ['head_1_joint', 'head_2_joint',
                       'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                       'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
                       'torso_lift_joint',
                       'hand_index_joint', 'hand_mrl_joint', 'hand_thumb_joint']

    def __init__(self, parent=None):
        super(myGUI, self).__init__(parent)
        self.rospack = rospkg.RosPack()
        path = self.rospack.get_path('learning_gui')
        print "path is: " + str(path)
        uic.loadUi(path + '/resources/tiago_learn_hand.ui', self)
        self.show()

        # self.update_gui_rate = 2.0  # Hz
        # # set subscribers
        # self.last_js = None
        # self.joint_states_sub = rospy.Subscriber(
        #     '/joint_states', JointState, self.js_cb, queue_size=1)
        # # the ones checked true
        # self.current_interesting_joints = self.get_enabled_joints()
        # self.play_motion_pub = rospy.Publisher(
        #     '/play_motion/goal', PlayMotionActionGoal, queue_size=1)

        # self.update_timer = QTimer(self)
        # self.update_timer.setInterval(1000.0 /
        #                               self.update_gui_rate)
        # self.update_timer.timeout.connect(self.update_current_joints_label)
        # self.update_timer.start()

        # # set callbacks to buttons (checkboxes are just monitored)
        # self.capture_button.clicked.connect(self.capture_joints)
        # self.upload_button.clicked.connect(self.upload_to_param_server)
        # self.play_button.clicked.connect(self.play_motion)



    def capture_joints(self):
        """
    open_hand:
      joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
      points:
      - positions: [-1.0, -1.0, -1.0]
        time_from_start: 0.0
      - positions: [0.0, 0.0, 0.0]
        time_from_start: 1.5
      meta:
        name: open_hand
        usage: demo
        description: 'open_hand'
        """

        motion_name = "change_me_motion_name"
        play_motion_str = "    " + motion_name + ":\n"
        play_motion_str += "      joints: " + \
            str(self.current_interesting_joints) + "\n"
        play_motion_str += "      points:\n"
        play_motion_str += "      - positions: "
        positions = []
        for j_name in self.current_interesting_joints:
            positions.append(round(get_joint_val(j_name, self.last_js), 3))
        play_motion_str += str(positions) + "\n"
        play_motion_str += "        time_from_start: 0.0\n"
        play_motion_str += """      meta:
        name: change_me_motion_name
        usage: demo
        description: 'change_me_motion_name'"""

        self.current_pose.setText(play_motion_str)
        # if nothing was written in the last field
        if self.full_motion.toPlainText() == "":
            extra_header = """play_motion:
  motions:\n"""
            self.full_motion.setText(extra_header + play_motion_str)

    def upload_to_param_server(self):
        if self.full_motion.toPlainText() != "":
            # To ease our life, create a tmp file to upload from it
            tmp_filename = '/tmp/capture_gui_tmp_yaml_file_to_upload_params.yaml'
            with open(tmp_filename, 'w') as f:
                f.write(self.full_motion.toPlainText())
            load_params_from_yaml(tmp_filename)
            rospy.loginfo("Loaded into param server the motion.")
        else:
            rospy.logerr(
                "Nothing in full motion field to upload to param server")
            return

    def play_motion(self):
        pmg = PlayMotionActionGoal()
        pmg.goal.motion_name = str(self.motion_name.toPlainText())
        pmg.goal.skip_planning = False
        rospy.loginfo("Sending goal: " + str(pmg.goal))
        self.play_motion_pub.publish(pmg)

    def js_cb(self, data):
        """
        :type data: JointState
        """
        self.last_js = data

    def get_enabled_joints(self):
        enabled_joints = []
        for j_name in self.joint_name_list:
            # this is like doing self.checkboxname.isChecked()
            if self.__dict__.get(j_name + "_c").isChecked():
                enabled_joints.append(j_name)
        rospy.logdebug("Found enabled joints: " + str(enabled_joints))
        return enabled_joints


if __name__ == '__main__':
    rospy.init_node('capture_gui')
    app = QtGui.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    window = myGUI()
    sys.exit(app.exec_())