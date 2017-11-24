#! /usr/bin/env python
import rospy
import roslib
import actionlib
import openravepy
import adapy
import numpy as np
import argparse
import logging
import prpy
import os
import IPython
from pynput import keyboard
import copy
import yaml
import os
import object_tracker.tracker as tk

def AddConstraintBoxes(env, robot, handedness='right', 
                        name_base="constraint_boxes_", 
                        visible=False):
    # Modifies environment to keep the robot inside a defined space
    # Does so by adding invisible boxes around the robot, which planners
    # avoid collisions with

    #add a box behind the robot
    box_behind = openravepy.RaveCreateKinBody(env,'')
    box_behind.SetName(name_base + 'behind')
    box_behind.InitFromBoxes(np.array([[0.,0.,0., 0.4, 0.1, 1.0]]), visible)
    env.Add(box_behind)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[1,3] = 0.57
    if handedness == 'right':
        T[0,3] += 0.25
    else:
        T[0,3] -= 0.25
    box_behind.SetTransform(T)


    #add a box above so we don't swing that way too high
    box_above = openravepy.RaveCreateKinBody(env,'')
    box_above.SetName(name_base + 'above')
    box_above.InitFromBoxes(np.array([[0.,0.,0., 0.5, 0.5, 0.1]]), visible)
    env.Add(box_above)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[0,3] += 0.25
    T[1,3] -= 0.25
    T[2,3] += 0.90
    box_above.SetTransform(T)


    box_left = openravepy.RaveCreateKinBody(env,'')
    box_left.SetName(name_base + 'left')
    box_left.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_left)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] += 0.9
    else:
        T[0,3] += 0.25
    T[1,3] = 0.25
    box_left.SetTransform(T)

    box_right = openravepy.RaveCreateKinBody(env,'')
    box_right.SetName(name_base + 'right')
    box_right.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_right)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] -= 0.25
    else:
        T[0,3] -= 0.9
    T[1,3] = 0.25
    box_right.SetTransform(T)

def readFromCamera():
    manip_april_config = rospy.get_param("Manipulator_AprilTag_config")
    april_tags_config = rospy.get_param("april_tags_config")
    refTagList=manip_april_config['tag_cal']
    obj_tags=april_tags_config['april_tags_id_list']
    print refTagList
    origin_tag = refTagList[0]
    x_tag = refTagList[1]
    y_tag = refTagList[2]
    positions = {}
    try:
        positions = tk.get_object_positions(origin_tag, 
                                            x_tag, y_tag, obj_tags, 
                                            timeout = 5)
        pos_offset = np.array(manip_april_config['pos_offset'])
        positions = {k: positions[k] + pos_offset for k in positions.keys()}
    except KeyError as e:
        rospy.logerr('Failed to get positions: {}'.format(e.message))
    for k in obj_tags:
        if k not in positions:
            positions[k] = None
    print positions
    return positions

def add_kinbody(KinbodyName):
    table = env.GetKinBody('table')
    tableConfig = table.GetConfigurationValues()
    tableHeight = tableConfig[3]
    
    kinbody = env.ReadKinBodyURI('objects/'+ KinbodyName[:-1] + '.kinbody.xml')
    kinbodyTrans = kinbody.GetTransform()
    
    tag = int(KinbodyName[-1])
    positions = readFromCamera()
    obj_pos = positions[tag]
    _x = obj_pos[0]
    _y = obj_pos[1]
    kinbodyTrans[0][3] = (_x)
    kinbodyTrans[1][3] = (_y)*(-1)
    kinbodyTrans[2][3] = tableHeight + 0.042
    # kinbodyTrans = np.dot(kinbodyTrans, rotz90)
    kinbody.SetTransform(kinbodyTrans)
    env.AddKinBody(kinbody)

def create_env():
    tables = env.ReadKinBodyURI('objects/table.kinbody.xml')
    rot90 = ([1.,0.,0.,0.],[0.,0.,-1.,0.],[0.,1.,0.,0.],[0.,0.,0.,1.])
    tableTrans = tables.GetTransform()
    tableTrans = np.dot(tableTrans,rot90)
    tableTrans[1][3] -= 0.3
    tables.SetTransform(tableTrans)
    env.AddKinBody(tables)
    tableConfig=tables.GetConfigurationValues()
    tableHeight= tableConfig[3]
    robotTrans = robot.GetTransform()
    robotTrans[2][3] = tableHeight + 0.02
    robotTrans[1][3] -= 0.3/2
    print tableHeight
    robot.SetTransform(robotTrans)
    AddConstraintBoxes(env,robot)
def bringToUser(kinbodyStr):
    finger_link_inds = []
    grab_link = None
    for ind,link in enumerate(robot.GetLinks()):
        if 'inger' in link.GetName():
            finger_link_inds.append(ind)
        if 'end_effector' in link.GetName():
            grab_link = link
    kinbody = env.GetKinBody(kinbodyStr[:-1])
    robot.arm.hand.CloseHand(1.2)
    robot.Grab(kinbody, grablink = grab_link, linkstoignore = finger_link_inds)
    config = np.array(
        [-1.27723448,
        -1.09628494, 
        -0.37344909, 
        -1.28876703,  
        2.14318495,
        3.44385185])
    plan = robot.arm.PlanToConfiguration(config, timeout = 10)
    robot.ExecutePath(plan)
    raw_input('Press enter to continue: ')
    robot.arm.hand.CloseHand(0)
    env.RemoveKinBody(kinbody)
    robot.arm.PlanToNamedConfiguration('home', execute = True)
def graspTSR(desiredKinbody):
    robot.arm.PlanToNamedConfiguration('home')
    robot.arm.hand.CloseHand(0)
    kinbody = env.GetKinBody(desiredKinbody[:-1])
    kinbodyTrans = kinbody.GetTransform()
    T0_w = kinbodyTrans
    Bw = np.zeros((6,2))
    if('fuze_bottle' in desiredKinbody):
        print "I got fuze bottle"
        Tw_e =  np.array([[ 0., 0., 1., -0.005],  
                               [1., 0., 0., 0],
                               [0., 1., 0., 0.05], 
                               [0., 0., 0., 1.]])
        Bw[2,:] = [0.0, 0.10] 
        Bw[5,:] = [-np.pi, np.pi]
    elif ('potted_meat_can' in desiredKinbody):
        print "I got potted meat can"
        Tw_e =  np.array([[ 0., 0., 1., 0.0],  
                              [1., 0., 0., 0],
                              [0., 1., 0., 0.03],
                              [0., 0., 0., 1.]])
        rot90 = ([1.,0.,0.,0.],[0.,0.,-1.,0.],[0.,1.,0.,0.],[0.,0.,0.,1.])
        Tw_e = np.dot(Tw_e, rot90)
    else:
        print "I got soup can "
        Tw_e =  np.array([[ 0., 0., 1., -0.005],  # desired offset between end-effector and object along x-axis
                               [1., 0., 0., 0],
                               [0., 1., 0., 0.01], # glass height
                               [0., 0., 0., 1.]])
        Bw[2,:] = [0.0, 0.04]
        Bw[5,:] = [-np.pi, np.pi]
    
    manip_idx = robot.GetActiveManipulatorIndex()
    grasp_tsr = prpy.tsr.TSR(T0_w = T0_w, 
                            Tw_e = Tw_e, 
                            Bw = Bw, 
                            manipindex = manip_idx)
    tsr_chain = prpy.tsr.TSRChain(sample_goal=True,
                                 sample_start=False, 
                                 constrain=False, 
                                 TSR=grasp_tsr)
    IPython.embed()

if __name__ == '__main__':
    rospy.init_node('manipulateObject') #this is the action
    env, robot = adapy.initialize(
        sim = True,
        attach_viewer = 'rviz'
    )
    create_env()
    objectList = ['potted_meat_can4', 'tomato_soup_can3', 'fuze_bottle2']
    for obj in objectList:
        add_kinbody(obj)  
    while True:
        id = int(raw_input("Enter ID {0, 1, 2}"))
        graspTSR(objectList[id])
        bringToUser(objectList[id])
        raw_input("Press Enter")
        add_kinbody(objectList[id])
    rospy.spin()

