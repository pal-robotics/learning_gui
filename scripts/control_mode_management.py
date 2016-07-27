#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 9/2/15

@author: sampfeiffer

gravity.py contains...
"""

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, ListControllersResponse

CONTROLLER_MANAGER_SWITCH_SRV = '/controller_manager/switch_controller'
CONTROLLER_MANAGER_LIST_SRV = '/controller_manager/list_controllers'


def is_controller_in_state(controller_name, desired_state,
                           list_controller_response=None):
    """
    Check if the controller is there and if its in it's desired state.
    E.G.: check if gravity_controller is there and is in state stopped
    :param controller_name:
    :param desired_state:
    :return:
    """
    if list_controller_response is None:
        list_srv = rospy.ServiceProxy(
            CONTROLLER_MANAGER_LIST_SRV, ListControllers)
        lcr = ListControllersRequest()
        list_controller_response = list_srv.call(lcr)
    for ctrler in list_controller_response.controller:
        if ctrler.name == controller_name and ctrler.state == desired_state:
            return True
    return False


def get_controllers_list():
    list_srv = rospy.ServiceProxy(CONTROLLER_MANAGER_LIST_SRV, ListControllers)
    lcr = ListControllersRequest()
    return list_srv.call(lcr)


def get_running_controller(list_controller_response):
    if is_controller_in_state('whole_body_kinematic_controler', 'running', list_controller_response):
        return 'whole_body_kinematic_controler'
    elif is_controller_in_state('gravity_compensation', 'running', list_controller_response):
        return 'gravity_compensation'
    elif is_controller_in_state('arm_controller', 'running', list_controller_response):
        return 'position'
    else:
        rospy.logerr(
            "Not any of 'whole_body_kinematic_controler' or 'gravity_compensation' or 'arm_controller' are running")
        return None


def go_to_gravity_compensation(switch_srv):
    scr = SwitchControllerRequest()
    scr.start_controllers.append('gravity_compensation')
    scr.start_controllers.append('wrist_controller')
    scr.stop_controllers.append('arm_controller')
    rospy.loginfo("Switch controllers: " + str(scr))
    resp = switch_srv.call(scr)
    if not resp.ok:
        rospy.logerr(
            "Could not switch. You may want to check rqt_console and try again.")


def go_to_position(switch_srv):
    scr = SwitchControllerRequest()
    scr.start_controllers.append('arm_controller')
    scr.stop_controllers.append('gravity_compensation')
    scr.stop_controllers.append('wrist_controller')
    rospy.loginfo("Switch controllers: " + str(scr))
    resp = switch_srv.call(scr)
    if not resp.ok:
        rospy.logerr(
            "Could not switch. You may want to check rqt_console and try again.")


def go_to_position_arm_head_torso_stop_wbc(switch_srv):
    scr = SwitchControllerRequest()
    scr.start_controllers.append('arm_controller')
    scr.start_controllers.append('head_controller')
    scr.start_controllers.append('torso_controller')
    scr.stop_controllers.append('whole_body_kinematic_controler')
    rospy.loginfo("Switch controllers: " + str(scr))
    resp = switch_srv.call(scr)
    if not resp.ok:
        rospy.logerr(
            "Could not switch. You may want to check rqt_console and try again.")


def go_to_position_arm_head_torso_stop_gravity(switch_srv):
    scr = SwitchControllerRequest()
    scr.start_controllers.append('arm_controller')
    scr.start_controllers.append('head_controller')
    scr.start_controllers.append('torso_controller')
    scr.stop_controllers.append('gravity_compensation')
    scr.stop_controllers.append('wrist_controller')
    rospy.loginfo("Switch controllers: " + str(scr))
    resp = switch_srv.call(scr)
    if not resp.ok:
        rospy.logerr(
            "Could not switch. You may want to check rqt_console and try again.")


def go_to_whole_body_kinematics(switch_srv):
    scr = SwitchControllerRequest()
    scr.start_controllers.append('whole_body_kinematic_controler')
    scr.stop_controllers.append('arm_controller')
    scr.stop_controllers.append('head_controller')
    scr.stop_controllers.append('torso_controller')
    rospy.loginfo("Switch controllers: " + str(scr))
    resp = switch_srv.call(scr)
    if not resp.ok:
        rospy.logerr(
            "Could not switch. You may want to check rqt_console and try again.")


def change_to_controller(controller_to_start):
    list_controller_response = get_controllers_list()
    from_controller = get_running_controller(list_controller_response)
    if controller_to_start.startswith('w'):  # wholebodycontrol
        to_controller = 'whole_body_kinematic_controler'
    elif controller_to_start.startswith('g'):  # position arm
        to_controller = 'gravity_compensation'
    elif controller_to_start.startswith('p'):  # gravity compensation
        to_controller = 'position'
    else:
        rospy.logerr("Given a controller to start that isn't one of:" +
                     "'[w]hole_body_kinematic_controler' or '[g]ravity_compensation' or '[p]osition'")
        return False

    if from_controller == to_controller:
        rospy.logwarn(
            "Asked to change to the mode we already are, doing nothing.")
        return False

    # position control (head, arm, torso).")
    rospy.loginfo(
        "Changing from '" + str(from_controller) + "' to  '" + to_controller + "'")
    switch_srv = rospy.ServiceProxy(
        CONTROLLER_MANAGER_SWITCH_SRV, SwitchController)
    if from_controller == 'whole_body_kinematic_controler':
        if to_controller == 'gravity_compensation':
            go_to_position_arm_head_torso_stop_wbc(switch_srv)
            go_to_gravity_compensation(switch_srv)
        elif to_controller == 'position':
            go_to_position_arm_head_torso_stop_wbc(switch_srv)
        else:
            rospy.logerr("Unexpected controller switch.")

    elif from_controller == 'gravity_compensation':
        if to_controller == 'whole_body_kinematic_controler':
            go_to_position(switch_srv)
            go_to_whole_body_kinematics(switch_srv)
        elif to_controller == 'position':
            go_to_position(switch_srv)
        else:
            rospy.logerr("Unexpected controller switch.")

    elif from_controller == 'position':
        if to_controller == 'gravity_compensation':
            go_to_gravity_compensation(switch_srv)
        elif to_controller == 'whole_body_kinematic_controler':
            go_to_whole_body_kinematics(switch_srv)
        else:
            rospy.logerr("Unexpected controller switch.")

    else:
        rospy.logerr("Unexpected controller switch.")

    rospy.loginfo("Done.")
