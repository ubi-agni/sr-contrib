#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, CITEC, Bielefeld University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Guillaume Walck <gwalck@techfak.uni-bielefeld.de>

"""
    generate and load one the following moveit yaml config file
    based on srdf on parameter server
    syntax : generate_load_moveit_config command [template_file]
    fake_controllers (no template)
    ompl_planning (template required)
    kinematics (template required)
    joint_limits (template required)
    """

import sys
import time
import rospy
from generate_moveit_config import generate_fake_controllers,\
    generate_ompl_planning, generate_kinematics, generate_joint_limits
from srdfdom.srdf import SRDF


if __name__ == '__main__':

    # detect the command to be executed
    if len(sys.argv) > 1:
        command = sys.argv[1]
        rospy.init_node('moveit_config_generator', anonymous=True)
        if command in ['fake_controllers', 'ompl_planning',
                       'kinematics', 'joint_limits']:
            NS = rospy.get_namespace()
            # wait for parameters
            while (not rospy.search_param('robot_description_semantic') and
                   not rospy.is_shutdown()):
                time.sleep(0.5)
                print "waiting for robot_description_semantic"
            # load the srdf from the parameter server
            full_param_name = rospy.search_param('robot_description_semantic')
            srdf_str = rospy.get_param(full_param_name)
            # parse it
            robot = SRDF.from_xml_string(srdf_str)
            # generate the desired yaml and load it.
            if command == "fake_controllers":
                generate_fake_controllers(robot, ns_=NS)
            elif command == "ompl_planning":
                # get the template file
                if len(sys.argv) > 2:
                    template_path = sys.argv[2]
                    # reject ROS internal parameters
                    # and detect termination
                    if (template_path.startswith("_") or
                            template_path.startswith("--")):
                        template_path = None
                    generate_ompl_planning(robot,
                                           template_path=template_path,
                                           ns_=NS)
                else:
                    print "ompl_planning generation requires a template\
                     file, none provided"
            elif command == "kinematics":
                # get the template file
                if len(sys.argv) > 2:
                    template_path = sys.argv[2]
                    if (template_path.startswith("_") or
                            template_path.startswith("--")):
                        template_path = None
                    generate_kinematics(robot,
                                        template_path=template_path,
                                        ns_=NS)
                else:
                    print "kinematics generation requires a template \
                    file, none provided"
                    sys.exit(1)
            elif command == "joint_limits":
                # get the template file
                if len(sys.argv) > 2:
                    template_path = sys.argv[2]
                    if (template_path.startswith("_") or
                            template_path.startswith("--")):
                        template_path = None
                    generate_joint_limits(robot,
                                          template_path=template_path,
                                          ns_=NS)
                else:
                    print "joint_limits generation requires a template \
                    file, none provided"
            else:
                print "wrong argument " + command

            print "Successfully loaded " + command + " params"
        else:
            print "Unrecognized command " + command + ". Choose among \
            fake_controllers, ompl_planning, kinematics joint_limits"
    else:
        print "Argument needed. Choose among fake_controllers, \
        ompl_planning, kinematics joint_limits"
