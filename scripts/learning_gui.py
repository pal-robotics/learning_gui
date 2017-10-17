#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 03/06/16
@author: Sammy Pfeiffer, Jordi Pages
@email: {sam.pfeiffer, jordi.pages}@pal-robotics.com
GUI to learn by demonstration for TIAGo
"""

import rospy
import rospkg
from sensor_msgs.msg import JointState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse
from simple_learning.srv import Learn, LearnRequest, LearnResponse
from control_mode_management import change_to_controller
from copy import deepcopy
import yaml
import sys
import signal
from PyQt4 import QtGui, uic
from PyQt4.QtCore import QTimer
import xml.dom.minidom


HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'

# TODO: Take sliders stuff from https://github.com/pal-robotics/reem_movements_creator/blob/master/scripts/joints_sliders.py
# TODO: link buttons to their actions
# TODO: manage the gravity comp mode where we can command the wrist
# TODO: manage putting in gray all the things that cannot be used (non clickable)
# TODO: Implement the learning part as a node that accepts service calls for the start and stopping
# TODO: take from https://github.com/awesomebytes/python_qt_tutorial the
# examples for saving and showing pop up errors


# Based on the implementation of joint_state_publisher
def get_joint_limits():
    """Gets the joint limits from the uploaded URDF
    /robot_description parameter"""
    # limits = {'arm_right_1_joint_min': -1.0,
    #           'arm_right_1_joint_max': 1.0}
    limits = {}
    description = rospy.get_param('robot_description')
    robot = xml.dom.minidom.parseString(
        description).getElementsByTagName('robot')[0]
    # Find all non-fixed joints
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'joint':
            jtype = child.getAttribute('type')
            if jtype == 'fixed' or jtype == 'floating':
                continue
            name = child.getAttribute('name')
            try:
                limit = child.getElementsByTagName('limit')[0]
                minval = float(limit.getAttribute('lower'))
                maxval = float(limit.getAttribute('upper'))
            except:
                rospy.logdebug(
                    "%s is not fixed, nor continuous, but limits are not specified!" % name)
                continue

            safety_tags = child.getElementsByTagName('safety_controller')
            if len(safety_tags) == 1:
                tag = safety_tags[0]
                if tag.hasAttribute('soft_lower_limit'):
                    minval = max(
                        minval, float(tag.getAttribute('soft_lower_limit')))
                if tag.hasAttribute('soft_upper_limit'):
                    maxval = min(
                        maxval, float(tag.getAttribute('soft_upper_limit')))
            limits[name + "_min"] = minval
            limits[name + "_max"] = maxval
    return limits


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


class LearningGUI(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(LearningGUI, self).__init__(parent)
        rospy.loginfo("Getting joint limits from URDF...")
        self.joint_limits = get_joint_limits()
        self.rospack = rospkg.RosPack()
        self.continuous_recording_mode_active = False
        path = self.rospack.get_path('learning_gui')
        print "path is: " + str(path)
        # Decide which GUI to use thanks to joint states...
        # set subscriber
        self.last_js = None
        self.joint_states_sub = rospy.Subscriber(
            '/joint_states', JointState, self.js_cb, queue_size=1)
        rospy.loginfo("Waiting for first joint states...")
        while self.last_js is None:
            rospy.sleep(0.2)
        rospy.loginfo("Guessing end effector in use...")
        if "hand_thumb_joint" in self.last_js.name:
            uic.loadUi(path + '/resources/tiago_learn_hand.ui', self)
            self.joint_names = ['head_1_joint', 'head_2_joint',
                                'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                                'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
                                'torso_lift_joint',
                                'hand_index_joint', 'hand_mrl_joint', 'hand_thumb_joint']
            self.eef = 'hand'
        elif "gripper_left_finger_joint" in self.last_js.name:
            uic.loadUi(path + '/resources/tiago_learn_gripper.ui', self)
            self.joint_names = ['head_1_joint', 'head_2_joint',
                                'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
                                'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint',
                                'torso_lift_joint',
                                'gripper_left_finger_joint', 'gripper_right_finger_joint']
            self.eef = 'gripper'
        self.set_sliders_limits()
        self.set_callbacks_sliders()
        self.controllers_srv = rospy.ServiceProxy('/controller_manager/list_controllers',
                                                  ListControllers)
        change_to_controller('position')
        self.controller_mode = 'position'
        self.gui_disabled_status = {'gravity_b': False,
                                    'position_b': True,
                                    'start_continuous': False,
                                    'stop_recording': True,
                                    'capture_waypoint': False,
                                    'play_05_b': True,
                                    'play_1_b': True,
                                    'play_2_b': True,
                                    'save_b': True,
                                    'arm_1_slider': False,  # For wrist controller
                                    'arm_2_slider': False,
                                    'arm_3_slider': False,
                                    'arm_4_slider': False}
        self.set_buttons_disabled()
        # check if we are in gravity to enable stuff related to it like
        # commanding with the wrist
        self.set_callbacks_buttons()
        self.show()

        self.update_gui_rate = 2.0  # Hz

        self.arm_pub = rospy.Publisher('/arm_controller/command',
                                       JointTrajectory,
                                       queue_size=1)

        self.wrist_pub = rospy.Publisher('/wrist_controller/command',
                                         JointTrajectory,
                                         queue_size=1)

        self.head_pub = rospy.Publisher('/head_controller/command',
                                        JointTrajectory,
                                        queue_size=1)

        self.torso_pub = rospy.Publisher('/torso_controller/command',
                                         JointTrajectory,
                                         queue_size=1)

        self.hand_pub = rospy.Publisher('/hand_controller/command',
                                        JointTrajectory,
                                        queue_size=1)

        self.gripper_pub = rospy.Publisher('/gripper_controller/command',
                                           JointTrajectory,
                                           queue_size=1)

        # self.play_motion_pub = rospy.Publisher(
        #     '/play_motion/goal', PlayMotionActionGoal, queue_size=1)

        self.update_timer = QTimer(self)
        self.update_timer.setInterval(1000.0 /
                                      self.update_gui_rate)
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start()

    def get_joint_limit_min(self, joint_name):
        return self.joint_limits[joint_name + '_min']

    def get_joint_limit_max(self, joint_name):
        return self.joint_limits[joint_name + '_max']

    def set_sliders_limits(self):
        for j_name in self.joint_names:
            # this is like doing self.checkboxname.isChecked()
            # [:-6] to remove '_joint'
            slider = self.__dict__.get(j_name[:-6] + "_slider")
            min_ = self.get_joint_limit_min(j_name)
            slider.setMinimum(min_ * 100)
            max_ = self.get_joint_limit_max(j_name)
            slider.setMaximum(max_ * 100)
            slider.setTickInterval(1)

    def set_callbacks_sliders(self):
        for joint_name in self.joint_names:
            method_name = self.add_cb_to_class_by_joint_name(joint_name)
            slider = self.__getattribute__(joint_name[:-6] + "_slider")
            slider.valueChanged.connect(self.__getattribute__(method_name))

    def set_buttons_disabled(self):
        for button_name in self.gui_disabled_status.keys():
            disabled = self.gui_disabled_status[button_name]
            self.__getattribute__(button_name).setDisabled(disabled)

    def set_callbacks_buttons(self):
        # Gravity comp
        self.gravity_b.clicked.connect(self.on_gravity)

        # Position
        self.position_b.clicked.connect(self.on_position)

        # START continuous mode
        self.start_continuous.clicked.connect(self.on_start_continuous)

        # STOP continuous mode
        self.stop_recording.clicked.connect(self.on_stop_recording)

        # Record waypoint
        self.capture_waypoint.clicked.connect(self.on_record_waypoint)

        # Play 0.5, 1x, 2x
        self.play_05_b.clicked.connect(self.on_play_05)
        self.play_1_b.clicked.connect(self.on_play_1)
        self.play_2_b.clicked.connect(self.on_play_2)

        # Save motion to file
        self.save_b.clicked.connect(self.on_save)

        # Checkboxes
        self.arm_1_cb.stateChanged.connect(self.check_all_arm)
        self.arm_2_cb.stateChanged.connect(self.check_all_arm)
        self.arm_3_cb.stateChanged.connect(self.check_all_arm)
        self.arm_4_cb.stateChanged.connect(self.check_all_arm)
        self.arm_5_cb.stateChanged.connect(self.check_all_arm)
        self.arm_6_cb.stateChanged.connect(self.check_all_arm)
        self.arm_7_cb.stateChanged.connect(self.check_all_arm)

        if self.eef == 'gripper':
            self.gripper_left_finger_cb.stateChanged.connect(self.check_all_gripper)
            self.gripper_right_finger_cb.stateChanged.connect(self.check_all_gripper)
        else:
            self.hand_index_cb.stateChanged.connect(self.check_all_hand)
            self.hand_mrl_cb.stateChanged.connect(self.check_all_hand)
            self.hand_thumb_cb.stateChanged.connect(self.check_all_hand)

        self.head_1_cb.stateChanged.connect(self.check_all_head)
        self.head_2_cb.stateChanged.connect(self.check_all_head)

    def check_all_arm(self, state):
        self.arm_1_cb.setCheckState(state)
        self.arm_2_cb.setCheckState(state)
        self.arm_3_cb.setCheckState(state)
        self.arm_4_cb.setCheckState(state)
        self.arm_5_cb.setCheckState(state)
        self.arm_6_cb.setCheckState(state)
        self.arm_7_cb.setCheckState(state)

    def check_all_head(self, state):
        self.head_1_cb.setCheckState(state)
        self.head_2_cb.setCheckState(state)

    def check_all_hand(self, state):
        self.hand_index_cb.setCheckState(state)
        self.hand_mrl_cb.setCheckState(state)
        self.hand_thumb_cb.setCheckState(state)

    def check_all_gripper(self, state):
        self.gripper_left_finger_cb.setCheckState(state)
        self.gripper_right_finger_cb.setCheckState(state)

    def on_gravity(self):
        change_to_controller('gravity')
        self.gui_disabled_status['position_b'] = False
        self.gui_disabled_status['gravity_b'] = True

    def on_position(self):
        change_to_controller('position')
        self.gui_disabled_status['position_b'] = True
        self.gui_disabled_status['gravity_b'] = False

    def on_start_continuous(self):
        req = LearnRequest()
        s = rospy.ServiceProxy('/learn_by_demo_start', Learn)
        joints = self.get_enabled_joints()
        print "start continuous recording of joints: " + str(joints)
        self.continuous_recording_mode_active = True
        req.joints_to_learn = joints
        req.record_single_waypoint = False
        s.call(req)
        self.gui_disabled_status['start_continuous'] = True
        self.gui_disabled_status['stop_recording'] = False
        self.gui_disabled_status['capture_waypoint'] = True
        self.gui_disabled_status['seconds_edit'] = True

    def on_stop_recording(self):
        req = LearnRequest()
        s = rospy.ServiceProxy('/learn_by_demo_stop', Learn)
        resp = s.call(req)
        if self.continuous_recording_mode_active:
            self.gui_disabled_status['start_continuous'] = False
        else:
            self.gui_disabled_status['capture_waypoint'] = False
        self.gui_disabled_status['stop_recording'] = True
        self.last_motion_text = resp.motion
        self.gui_disabled_status['play_05_b'] = False
        self.gui_disabled_status['play_1_b'] = False
        self.gui_disabled_status['play_2_b'] = False
        self.gui_disabled_status['save_b'] = False
        self.gui_disabled_status['seconds_edit'] = False

    def on_record_waypoint(self):
         req = LearnRequest()
         s = rospy.ServiceProxy('/learn_by_demo_start', Learn)
         joints = self.get_enabled_joints()
         print "record current status of joints: " + str(joints)
         req.joints_to_learn = joints
         req.record_single_waypoint = True
         req.seconds_for_each_waypoint = float(self.seconds_edit.toPlainText())
         rospy.loginfo("Seconds given for each waypoint: " + str(req.seconds_for_each_waypoint))
         self.continuous_recording_mode_active = False
         self.gui_disabled_status['start_continuous'] = True
         self.gui_disabled_status['stop_recording'] = True
         self.gui_disabled_status['capture_waypoint'] = True
         self.gui_disabled_status['seconds_edit'] = True
         s.call(req)
         self.gui_disabled_status['stop_recording'] = False
         self.gui_disabled_status['capture_waypoint'] = False


    def on_play_05(self):
        change_to_controller('position')
        pm = SimpleActionClient('/play_motion', PlayMotionAction)
        pm.wait_for_server()
        pmg = PlayMotionGoal()
        pmg.motion_name = 'LBD_HALFX'
        pm.send_goal(pmg)
        pm.wait_for_result(rospy.Duration(0.1))

    def on_play_1(self):
        change_to_controller('position')
        pm = SimpleActionClient('/play_motion', PlayMotionAction)
        pm.wait_for_server()
        pmg = PlayMotionGoal()
        pmg.motion_name = 'LBD_1X'
        pm.send_goal(pmg)
        pm.wait_for_result(rospy.Duration(0.1))

    def on_play_2(self):
        change_to_controller('position')
        pm = SimpleActionClient('/play_motion', PlayMotionAction)
        pm.wait_for_server()
        pmg = PlayMotionGoal()
        pmg.motion_name = 'LBD_2X'
        pm.send_goal(pmg)
        pm.wait_for_result(rospy.Duration(0.1))

    def on_save(self):
        # self.last_motion_text has the motion in text
        # get from my qt tutorial how to open a save window
        # path = QtGui.QFileDialog.getOpenFileName()
        path = QtGui.QFileDialog.getSaveFileName()
        if path != '':
            # get the motion from param server
            param = rospy.get_param('/play_motion/motions/LBD_1X')
            d = {'play_motion': {'motions': {'LBD_1X': param}}}
            text_motion = yaml.dump(d)
            with open(path, 'w') as f:
                f.write(text_motion)

    def add_cb_to_class_by_joint_name(self, joint_name):
        # Create dynamically a method to be called
        def make_method(joint_name):
            def _method():
                # print "Cb for joint '" + parameter_name + "' called."
                # print "We got value: " + str(new_value)
                new_value = self.__getattribute__(
                    joint_name[:-6] + "_slider").value()
                new_value = float(new_value / 100.0)

                # TODO: update slider and spinbox with new value
                self.send_goal(joint_name, new_value)

            return _method

        method_name = "callback_method_for_" + joint_name
        cb_method = make_method(joint_name)
        setattr(self, method_name, cb_method)
        return method_name


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

    def js_cb(self, data):
        """
        :type data: JointState
        """
        self.last_js = data

    def get_enabled_joints(self):
        enabled_joints = []
        for j_name in self.joint_names:
            # this is like doing self.checkboxname.isChecked()
            # [:-6] to remove '_joint'
            if self.__dict__.get(j_name[:-6] + "_cb").isChecked():
                enabled_joints.append(j_name)
        rospy.loginfo("Found enabled joints: " + str(enabled_joints))
        return enabled_joints

    def create_goal_for(self, joint_name, new_value):
        print "Creating a goal for... " + str(joint_name)
        jt = JointTrajectory()
        if "head" in joint_name:
            jt.joint_names = ['head_1_joint', 'head_2_joint']
        elif "torso" in joint_name:
            jt.joint_names = ['torso_lift_joint']
        elif "arm" in joint_name:
            if self.controller_mode == 'position':
                jt.joint_names = ['arm_1_joint',
                                  'arm_2_joint',
                                  'arm_3_joint',
                                  'arm_4_joint',
                                  'arm_5_joint',
                                  'arm_6_joint',
                                  'arm_7_joint']
            elif self.controller_mode == 'gravity':
                jt.joint_names = ['arm_5_joint',
                                  'arm_6_joint',
                                  'arm_7_joint']

        elif "hand" in joint_name:
            jt.joint_names = ['hand_thumb_joint',
                              'hand_index_joint',
                              'hand_mrl_joint']
        elif "gripper" in joint_name:
            jt.joint_names = ['gripper_left_finger_joint',
                              'gripper_right_finger_joint']

        jtp = JointTrajectoryPoint()
        for j_name in jt.joint_names:
            if j_name == joint_name:
                jtp.positions.append(new_value)
            else:
                jtp.positions.append(self.get_joint_position(j_name))

        # TODO: maybe tune time for joint groups too, fingers may be too slow
        # Goals will take 1.0 seconds + a part relative on how much it should
        # move
        time_for_goal = 1.0 + \
            abs(self.get_joint_position(joint_name) - new_value) * 3.0
        jtp.time_from_start = rospy.Duration(time_for_goal)

        jt.points.append(jtp)

        return jt

    def get_joint_position(self, joint_name):
        if self.last_js is None:
            return

        idx = self.last_js.name.index(joint_name)
        if idx == -1:
            rospy.logerr(
                "joint: " + joint_name + " is not in last joint states")
        return self.last_js.position[idx]

    def send_goal(self, joint_name, new_value):
        if self.last_js is None:
            return
        goal = self.create_goal_for(joint_name, new_value)
        pub = self.get_pub_for(joint_name)
        pub.publish(goal)

    def get_pub_for(self, joint_name):
        group = joint_name.split('_')[0]
        if group == 'arm':
            if self.controller_mode == 'gravity':
                return self.wrist_pub
        return self.__getattribute__(group + '_pub')

    def update_gui(self, *args):
        if self.last_js is None:
            return
        for joint in self.joint_names:
            # Get the joint position
            position = self.get_joint_position(joint)
            # Get the slider, and update the pose
            slider = self.__getattribute__(joint[:-6] + "_slider")
            # Block signals to not send goals on joint states updates
            slider.blockSignals(True)
            slider.setValue(int(position * 100))
            slider.blockSignals(False)

        self.set_buttons_disabled()
        self.get_current_controller()

    def get_current_controller(self):
        req = ListControllersRequest()
        resp = self.controllers_srv.call(req)
        for c in resp.controller:
            if c.name == 'wrist_controller':
                if c.state == 'running':
                    self.controller_mode = 'gravity'
                    self.gui_disabled_status['arm_1_slider'] = True
                    self.gui_disabled_status['arm_2_slider'] = True
                    self.gui_disabled_status['arm_3_slider'] = True
                    self.gui_disabled_status['arm_4_slider'] = True
            elif c.name == 'arm_controller':
                if c.state == 'running':
                    self.controller_mode = 'position'
                    self.gui_disabled_status['arm_1_slider'] = False
                    self.gui_disabled_status['arm_2_slider'] = False
                    self.gui_disabled_status['arm_3_slider'] = False
                    self.gui_disabled_status['arm_4_slider'] = False


if __name__ == '__main__':
    rospy.init_node('capture_gui')
    app = QtGui.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    window = LearningGUI()
    sys.exit(app.exec_())
