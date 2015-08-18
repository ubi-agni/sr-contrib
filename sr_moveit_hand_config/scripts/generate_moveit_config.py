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
generate_moveit_config provides:
    generate_fake_controllers : generage a fake controllers config file
    generate_ompl_planning : generate ompl_planning config file
    generate_kinematics : generate kinematics config file
    generate_joint_limits : generate joint limits config file
"""

import argparse
import rosparam
import yaml
import re
from srdfdom.srdf import SRDF


def yaml_reindent(in_str, numspaces):
    """
    Add numspaces space in fron of each line of the input string
        @param in_str: input string
        @type in_str:  str
        @param numspaces: number of spaces to indent the string with
        @type numspaces:  int
        @return s_intend: indented string
    """
    s_indent = "\n".join((numspaces * " ") + i for i in in_str.splitlines())
    return s_indent


def find_prefix(robot):
    """
    Find the prefix using the always available shadow_hand group name
        @param robot: parsed SRDF
        @type robot:  SRDF object
        @return prefix: prefix in a string
    """
    prefix = ""
    for key in robot.group_map:
        if key.endswith("shadow_hand"):
            prefix = key[0:key.find("shadow_hand")]
    print "found prefix:", prefix
    return prefix


def upload_output_params(upload_str, output_path=None, ns_=None):
    """
    Upload or output the input string on the correct param ns or file
        @param upload_str: string to be uploaded or written
        @type upload_str:  str
        @param output_path: output path of file to be written.
        Upload if None
        @type output_path:  str
        @param ns_: namespace to use when uploading to param server
        @type ns_:  str
    """
    if output_path is None:
        paramlist = rosparam.load_str(upload_str, "generated",
                                      default_namespace=ns_)
        for params, namespace in paramlist:
            rosparam.upload_params(namespace, params)
    else:
        file_writer = open(output_path, "wb")
        file_writer.write(upload_str)
        file_writer.close()


def generate_fake_controllers(robot, output_path=None, ns_=None):
    """
    Generate fake_controller yaml and direct it to file
    or load it to parameter server.
        @param robot: Parsed SRDF
        @type  robot: SRDF object
        @param output_path: file_path to save the generated data in,
        will load on parameter server if empty
        @type  output_path: str
        @param ns_: namespace
        @type  ns_: str
    """
    output_str = ""
    output_str += "controller_list:\n"
    # for each group
    for group in robot.groups:
        controller_name = "  - name: fake_" + group.name\
                          + "_controller\n"
        output_str += controller_name
        output_str += "    joints:\n"
        if len(group.joints) == 0:
            output_str += "      []\n"
        else:
            for joint in group.joints:
                output_str += "      - " + joint.name + "\n"
    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_ompl_planning(robot,
                           template_path="ompl_planning_template.yaml",
                           output_path=None, ns_=None):
    '''
    Generate ompl_planning yaml and direct it to file
    or load it to parameter server.
        @param robot: Parsed SRDF
        @type  robot: SRDF object
        @param template_path: file_path to the req. yaml template
        @type  template_path: str
        @param output_path: file_path to save the generated data in,
        will load on parameter server if empty
        @type  output_path: str
        @param ns_: namespace
        @type  ns_: str
    '''
    output_str = ""

    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    output_str += "planner_configs:\n"
    output_str += yaml_reindent(yaml.dump(
                                yamldoc["planner_configs"],
                                default_flow_style=False,
                                allow_unicode=True),
                                2)
    output_str += "\n"
    # find prefix
    prefix = find_prefix(robot)
    if prefix:
        proj_eval_re = re.compile(r'joints\(([TFMRLW][FHR]J[0-5]),([TFMRLW][FHR]J[0-5])\)')
    # for each group
    for group in robot.groups:
        # strip prefix if any
        group_name = group.name[len(prefix):]
        if group_name in yamldoc:
            output_str += group.name+":\n"
            group_config = yamldoc[group_name]
            # prepend prefix on projection_evaluator
            if prefix:
                if "projection_evaluator" in group_config:
                    proj_eval = group_config["projection_evaluator"]
                    proj_eval.strip()
                    proj_eval_new = proj_eval_re.sub(r'joints(' +
                                                        prefix +
                                                        r'\g<1>,' +
                                                        prefix +
                                                        r'\g<2>)',
                                                        proj_eval)
                    group_config["projection_evaluator"] = proj_eval_new
            group_dump = yaml.dump(group_config,
                                   default_flow_style=False,
                                   allow_unicode=True)
            output_str += yaml_reindent(group_dump, 2)
            output_str += "\n"
    stream.close()
    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_kinematics(robot, template_path="kinematics_template.yaml",
                        output_path=None, ns_=None):
    """
    Generate kinematics yaml and direct it to file
    or load it to parameter server.
        @param srdf: Parsed SRDF
        @type  srdf: XML object
        @param template_path: file_path to the req. yaml template
        (biotac version will be loaded automatically)
        @type  template_path: str
        @param output_path: file_path to save the generated data in,
        will load on parameter server if empty
        @type  output_path: str
        @param ns_: namespace
        @type  ns_: str
    """
    output_str = ""
    # open template file
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    stream.close()

    # open biotac template file
    biotac_template_path = template_path[
        0:template_path.find("_template")]+"_biotac_template.yaml"

    stream = open(biotac_template_path, 'r')
    yamldocbiotac = yaml.load(stream)
    stream.close()

    # find prefix
    prefix = find_prefix(robot)

    # find full hand key name
    sh_group = None
    for group in robot.groups:
        if group.name.endswith("shadow_hand"):
            sh_group = group
            break

    # detect biotac fingers
    is_bio = {"first_finger": False,
              "middle_finger": False,
              "ring_finger": False,
              "little_finger": False,
              "thumb": False}

    for mylink in sh_group.links:
        link_name = mylink.name
        if not is_bio["first_finger"] and link_name.endswith("ffbiotac"):
            is_bio["first_finger"] = True
        if not is_bio["middle_finger"] and link_name.endswith("mfbiotac"):
            is_bio["middle_finger"] = True
        if not is_bio["ring_finger"] and link_name.endswith("rfbiotac"):
            is_bio["ring_finger"] = True
        if not is_bio["little_finger"] and link_name.endswith("lfbiotac"):
            is_bio["little_finger"] = True
        if not is_bio["thumb"] and link_name.endswith("thbiotac"):
            is_bio["thumb"] = True

    # for each group
    for group in robot.groups:
        kinematics_config = None
        # strip prefix if any
        group_name = group.name[len(prefix):]
        # check for biotac link for this group
        if is_bio.get(group_name):
            if group_name in yamldocbiotac:
                kinematics_config = yamldocbiotac[group_name]
        else:
            if group_name in yamldoc:
                kinematics_config = yamldoc[group_name]

        if kinematics_config is not None:
            output_str += group.name+":\n"
            output_str += yaml_reindent(yaml.dump(kinematics_config,
                                        default_flow_style=False,
                                        allow_unicode=True), 2)
            output_str += "\n"

    # load on param server or output to file
    upload_output_params(output_str, output_path, ns_)


def generate_joint_limits(robot,
                          template_path="joint_limits_template.yaml",
                          output_path=None, ns_=None):
    """
    Generate joint_limits yaml and direct it to file
    or load it to parameter server.
        @param robot: Parsed SRDF
        @type  robot: SRDF object
        @param template_path: file_path to the required yaml template file
        @type  template_path: str
        @param output_path: file_path to save the generated data in,
        will load on parameter server if empty
        @type  output_path: str
        @param ns_: namespace
        @type  ns_: str
    """
    output_str = ""
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    output_str += "joint_limits:\n"
    # find full hand key name
    for key in robot.group_map:
        if key.endswith("shadow_hand"):
            group_name = key

    if group_name is not None:
        # for each joint in full hand group
        for joint in robot.group_map[group_name].joints:
            joint_name = joint.name[-4:]
            #print "testing joint name ",joint_name," out of ",joint.name
            if joint_name in yamldoc["joint_limits"]:
                joint_limits_config = yamldoc["joint_limits"][joint_name]
                output_str += "  " + joint.name + ":\n"
                joint_limits_dump = yaml.dump(
                    joint_limits_config,
                    default_flow_style=False,
                    allow_unicode=True)
                output_str += yaml_reindent(joint_limits_dump, 4)
                output_str += "\n"
        stream.close()
        # load on param server or output to file
        upload_output_params(output_str, output_path, ns_)


if __name__ == '__main__':

    PARSER = argparse.ArgumentParser(usage='Load an SRDF file')
    PARSER.add_argument('file', type=argparse.FileType('r'), nargs='?',
                        default=None,
                        help='File to load. Use - for stdin')
    ARGS = PARSER.parse_args()

    if ARGS.file is not None:
        ROBOT = SRDF.from_xml_string(ARGS.file.read())
        generate_fake_controllers(ROBOT,
                                  output_path="fake_controllers.yaml")
        generate_ompl_planning(ROBOT,
                               "ompl_planning_template.yaml",
                               output_path="ompl_planning.yaml")
        generate_kinematics(ROBOT,
                            "kinematics_template.yaml",
                            output_path="kinematics.yaml")
        generate_joint_limits(ROBOT,
                              "joint_limits_template.yaml",
                              output_path="joint_limit.yaml")
    else:
        print "No file SRDF provided"
