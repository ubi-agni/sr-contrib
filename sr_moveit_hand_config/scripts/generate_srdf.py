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
    generate the srdf according to the urdf
    syntax  generate_srdf <srdf.xacro filename> [output filename]
    Note : The srdf.xacro filename should be without _prefix,
    as the name with prefix is generated from the one without if needed
"""

import sys
from urdf_parser_py.urdf import URDF
import xacro
import rospy
from xml.dom.minidom import parse


if __name__ == '__main__':

    if len(sys.argv) > 1:
        srdf_xacro_filename = sys.argv[1]

        rospy.init_node('srdf_generator', anonymous=True)

        while not rospy.has_param('robot_description'):
            rospy.sleep(0.5)
            print "waiting for robot_description"

        # load the srdf from the parameter server
        urdf_str = rospy.get_param('robot_description')
        # parse it
        robot = URDF.from_xml_string(urdf_str)
        # find which version of the hand should be generated
        # (which fingers) also find the prefix if any
        extracted_prefix = False
        prefix = ""
        ff = mf = rf = lf = th = False
        ffbio = mfbio = rfbio = lfbio = thbio = False
        is_lite = True
        for key in robot.joint_map:
            # any joint is supposed to have the same prefix
            # and a joint name with 4 chars
            if not extracted_prefix:
                prefix = key[0:-4]
                print "found prefix:", prefix
                extracted_prefix = True

            if not ff and key.endswith("FFJ4"):
                ff = True
            if not mf and key.endswith("MFJ4"):
                mf = True
            if not rf and key.endswith("RFJ4"):
                rf = True
            if not lf and key.endswith("LFJ4"):
                lf = True
            if not th and key.endswith("THJ4"):
                th = True
            if is_lite and key.endswith("WRJ2"):
                is_lite = False
        print "found fingers (ff mf rf lf th)", (ff, mf, rf, lf, th)

        for key in robot.link_map:
            if not ffbio and key.endswith("ffbiotac"):
                ffbio = True
            if not mfbio and key.endswith("mfbiotac"):
                mfbio = True
            if not rfbio and key.endswith("rfbiotac"):
                rfbio = True
            if not lfbio and key.endswith("lfbiotac"):
                lfbio = True
            if not thbio and key.endswith("thbiotac"):
                thbio = True

        print "found bio fingers (ff mf rf lf th)", (ffbio, mfbio,
                                                     rfbio, lfbio, thbio)

        #TODO extract the robot name
        # selection the correct hand version through arg substitution
        mappings = dict(robot_name='shadowhand_motor',
                        ff=str(int(ff)), mf=str(int(mf)),
                        rf=str(int(rf)), lf=str(int(lf)),
                        th=str(int(th)), is_lite=str(int(is_lite)),
                        ff_bio=str(int(ffbio)), mf_bio=str(int(mfbio)),
                        rf_bio=str(int(rfbio)), lf_bio=str(int(lfbio)),
                        th_bio=str(int(thbio)))
        if prefix:
            mappings.update(prefix=prefix)
            # the prefix version of the srdf_xacro must be loaded
            srdf_xacro_filename = srdf_xacro_filename.replace(
                ".srdf.xacro", "_prefix.srdf.xacro")
            print "file loaded ", srdf_xacro_filename

        # open and parse the xacro.srdf file
        srdf_xacro_file = open(srdf_xacro_filename, 'r')
        srdf_xacro_xml = parse(srdf_xacro_file)

        # expand the xacro
        xacro.process_doc(srdf_xacro_xml, mappings=mappings)

        if len(sys.argv) > 2:
            OUTPUT_PATH = sys.argv[2]
            # reject ROS internal parameters and detect termination
            if (OUTPUT_PATH.startswith("_") or
               OUTPUT_PATH.startswith("--")):
                OUTPUT_PATH = None
        else:
            OUTPUT_PATH = None

        # Upload or output the input string
        # on the correct param namespace or file
        if OUTPUT_PATH is None:
            print " Loading SRDF on parameter server"
            robot_description_param = rospy.resolve_name(
                'robot_description') + "_semantic"
            print "semantic param ", robot_description_param
            rospy.set_param(robot_description_param,
                            srdf_xacro_xml.toprettyxml(indent=' '))
        else:
            print " Writing SRDF to file ", OUTPUT_PATH
            FW = open(OUTPUT_PATH, "wb")
            FW.write(srdf_xacro_xml.toprettyxml(indent=' '))
            FW.close()

        srdf_xacro_file.close()
    else:
        print "No srdf.xacro file provided"
